#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <LoRa.h>

// ════════════════════════════════════════════════════════════
//  HARDWARE & CONFIGURATION
// ════════════════════════════════════════════════════════════
#define NODE_ID          "NODE_03"
#define LOG_INTERVAL_MS   180000

#define GPS_RX           27
#define GPS_TX           14
#define GPS_BAUD         9600
#define GPS_WAIT_MS      2000
#define GPS_LED_PIN      2

#define SENSOR_RX        16   // 7-in-1 on UART2
#define SENSOR_TX        17
#define RS485_DE         4
#define SENSOR_BAUD      4800 
#define SENSOR_TIMEOUT   500

// New SPI LoRa Pins
#define LORA_NSS         5
#define LORA_RST         32
#define LORA_DIO0        21

// Season: moisture + temp from sensor
#define RAINY_MOISTURE_MIN  80.0f
#define RAINY_TEMP_MAX      25.0f

// ════════════════════════════════════════════════════════════
//  DATA STRUCTURE
// ════════════════════════════════════════════════════════════
struct SoilData {
  float moisture;    // %
  float temperature; // °C
  int   ec;          // µS/cm
  float ph;
  int   nitrogen;    // mg/kg
  int   phosphorus;  // mg/kg
  int   potassium;   // mg/kg
  bool  valid;
};

HardwareSerial gpsSerial(1);
HardwareSerial uart2(2);
TinyGPSPlus    gps;

unsigned long lastLog   = 0;
unsigned long lastBlink = 0;
bool          ledState  = false;

// ════════════════════════════════════════════════════════════
//  CRC-16 Modbus
// ════════════════════════════════════════════════════════════
uint16_t calculateCRC(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else              { crc >>= 1; }
    }
  }
  return crc;
}

// ════════════════════════════════════════════════════════════
//  7-in-1 Sensor
// ════════════════════════════════════════════════════════════
SoilData readSensor() {
  SoilData d = {0, 0, 0, 0, 0, 0, 0, false};

  uart2.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX, SENSOR_TX);
  delay(50);
  while (uart2.available()) uart2.read();

  const uint8_t request[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};

  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(100);
  uart2.write(request, sizeof(request));
  uart2.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_DE, LOW);

  uint8_t buf[19];
  uint8_t idx   = 0;
  uint32_t start = millis();
  while (millis() - start < SENSOR_TIMEOUT && idx < 19) {
    if (uart2.available()) buf[idx++] = uart2.read();
  }

  // We no longer need to close uart2 or toggle SENSOR_TX as an input 
  // since LoRa is now on SPI, not UART2.
  
  if (idx < 19) {
    Serial.printf("  [7-in-1] Timeout — got %d/19 bytes. Check wires.\n", idx);
    return d;
  }

  if (buf[0] != 0x01 || buf[1] != 0x03 || buf[2] != 0x0E) {
    Serial.printf("  [7-in-1] Bad header: %02X %02X %02X\n", buf[0], buf[1], buf[2]);
    return d;
  }

  uint16_t calcCRC = calculateCRC(buf, 17);
  uint16_t recvCRC = buf[17] | (buf[18] << 8); 
  if (calcCRC != recvCRC) {
    Serial.printf("  [7-in-1] CRC mismatch: calc 0x%04X  recv 0x%04X\n", calcCRC, recvCRC);
    return d;
  }

  d.moisture    = ((buf[3]  << 8) | buf[4])  / 10.0f;
  d.temperature = ((buf[5]  << 8) | buf[6])  / 10.0f;
  d.ec          =  (buf[7]  << 8) | buf[8];
  d.ph          = ((buf[9]  << 8) | buf[10]) / 10.0f;
  d.nitrogen    =  (buf[11] << 8) | buf[12];
  d.phosphorus  =  (buf[13] << 8) | buf[14];
  d.potassium   =  (buf[15] << 8) | buf[16];
  d.valid       = true;

  Serial.printf("  [7-in-1] OK — M:%.1f%% T:%.1f°C EC:%d pH:%.1f N:%d P:%d K:%d\n",
                d.moisture, d.temperature, d.ec, d.ph,
                d.nitrogen, d.phosphorus, d.potassium);
  return d;
}

// ════════════════════════════════════════════════════════════
//  GPS
// ════════════════════════════════════════════════════════════
bool readGPS() {
  unsigned long start = millis();
  while (millis() - start < GPS_WAIT_MS) {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) return true;
  }
  return gps.location.isValid();
}

void handleGPSLed() {
  if (gps.location.isValid()) { digitalWrite(GPS_LED_PIN, HIGH); return; }
  if (millis() - lastBlink >= 200) {
    lastBlink = millis();
    digitalWrite(GPS_LED_PIN, ledState = !ledState);
  }
}

String getTimestamp() {
  if (gps.date.isValid() && gps.time.isValid()) {
    char buf[22];
    sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
            gps.date.year(),  gps.date.month(),  gps.date.day(),
            gps.time.hour(),  gps.time.minute(), gps.time.second());
    return String(buf);
  }
  unsigned long s = millis() / 1000;
  char buf[20];
  sprintf(buf, "UPTIME %02lu:%02lu:%02lu", s / 3600, (s % 3600) / 60, s % 60);
  return String(buf);
}

String classifySeason(float moisture, float temperature) {
  return (moisture > RAINY_MOISTURE_MIN && temperature < RAINY_TEMP_MAX)
         ? "Rainy" : "Dry";
}

// ════════════════════════════════════════════════════════════
//  SPI LoRa TX (Updated for Ai-Thinker Ra-02)
// ════════════════════════════════════════════════════════════
bool sendLoRa(const SoilData &s, float lat, float lng, float alt, const String &sats, const String &season) {

  String pkt = String(NODE_ID)          + "," +
               String(s.moisture, 1)    + "," +
               String(s.temperature, 1) + "," +
               s.ec                     + "," +
               String(s.ph, 1)          + "," +
               s.nitrogen               + "," +
               s.phosphorus             + "," +
               s.potassium              + "," +
               String(lat, 6)           + "," +
               String(lng, 6)           + "," +
               String(alt, 1)           + "," +
               sats                     + "," +
               season;

  Serial.println("\n[LoRa] Sending: " + pkt);

  // Send packet using standard LoRa library over SPI
  LoRa.beginPacket();
  LoRa.print(pkt);
  LoRa.endPacket();

  Serial.printf("[LoRa] Sent (%d bytes).\n", pkt.length());
  return true; 
}

// ════════════════════════════════════════════════════════════
//  Print Readings
// ════════════════════════════════════════════════════════════
void printReadings(const SoilData &s, bool gpsOk, float lat, float lng, float alt, const String &sats, const String &season) {

  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.printf( "║   %s — SENSOR READINGS           ║\n", NODE_ID);
  Serial.println( "╚══════════════════════════════════════════╝");
  Serial.println("  Timestamp   : " + getTimestamp());
  Serial.println("  ────────────────────────────────────────");

  if (s.valid) {
    Serial.printf("  Moisture    : %.1f %%\n", s.moisture);
    Serial.print ("  Soil Status : ");
    if      (s.moisture < 20) Serial.println("DRY  — Consider watering!");
    else if (s.moisture < 50) Serial.println("SLIGHTLY MOIST");
    else if (s.moisture < 75) Serial.println("MOIST — OPTIMAL");
    else                      Serial.println("WET  — Reduce watering!");

    Serial.println("  ────────────────────────────────────────");
    Serial.printf("  Temperature : %.1f C\n",   s.temperature);
    Serial.printf("  EC          : %d us/cm\n", s.ec);
    Serial.printf("  pH          : %.1f\n",     s.ph);
    Serial.println("  ────────────────────────────────────────");
    Serial.printf("  Nitrogen  N : %d mg/kg\n", s.nitrogen);
    Serial.printf("  Phosphorus P: %d mg/kg\n", s.phosphorus);
    Serial.printf("  Potassium K : %d mg/kg\n", s.potassium);
  } else {
    Serial.println("  7-in-1      : ERROR — No valid response");
  }

  Serial.println("  ────────────────────────────────────────");
  if (gpsOk) {
    Serial.printf("  Latitude    : %.6f\n",   lat);
    Serial.printf("  Longitude   : %.6f\n",   lng);
    Serial.printf("  Altitude    : %.1f m\n", alt);
    Serial.printf("  Satellites  : %s\n",     sats.c_str());
    if (gps.time.isValid())
      Serial.printf("  UTC Time    : %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.printf("  Maps Link   : https://maps.google.com/?q=%.6f,%.6f\n", lat, lng);
  } else {
    Serial.println("  GPS         : Searching — LED blinking");
    Serial.printf ("  GPS chars   : %lu processed\n", gps.charsProcessed());
  }

  Serial.println("  ────────────────────────────────────────");
  Serial.printf( "  Season      : %s\n", season.c_str());
  Serial.println("  ══════════════════════════════════════════");
}

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  pinMode(RS485_DE,    OUTPUT); digitalWrite(RS485_DE,    LOW);
  pinMode(GPS_LED_PIN, OUTPUT); digitalWrite(GPS_LED_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==========================================");
  Serial.printf( "   %s — SPI LoRa Agri Node\n", NODE_ID);
  Serial.println("==========================================\n");

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // Initialize SPI LoRa
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("[INIT] SPI LoRa initialization failed. Check wiring!");
    while (1); 
  }
  Serial.println("[INIT] LoRa initialized at 433MHz.");
  
  Serial.println("[INIT] Ready! Logging every 3 minutes.\n");
  lastLog = millis() - LOG_INTERVAL_MS;
}

// ════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  handleGPSLed();

  if (millis() - lastLog < LOG_INTERVAL_MS) return;
  lastLog = millis();

  Serial.println("\n[TX] === Reading Sensors ===");

  SoilData soil = readSensor();
  bool   gpsOk   = readGPS();
  float  latVal  = gpsOk ? (float)gps.location.lat()  : 0.0f;
  float  lngVal  = gpsOk ? (float)gps.location.lng()  : 0.0f;
  float  altVal  = (gpsOk && gps.altitude.isValid())  ? (float)gps.altitude.meters() : 0.0f;
  String satsStr = (gpsOk && gps.satellites.isValid())? String(gps.satellites.value()) : "ERR";
  String season = classifySeason(soil.moisture, soil.temperature);

  printReadings(soil, gpsOk, latVal, lngVal, altVal, satsStr, season);

  Serial.println("[TX] Sending via SPI LoRa...");
  bool loraSent = sendLoRa(soil, latVal, lngVal, altVal, satsStr, season);

  Serial.println("\n[TX] ── Summary ─────────────────────");
  Serial.printf("[TX]   LoRa     : %s\n",    loraSent  ? "SENT ✓"  : "FAILED ✗");
  Serial.printf("[TX]   7-in-1   : %s\n",    soil.valid? "OK ✓"    : "ERROR ✗");
  Serial.printf("[TX]   Moisture : %.1f %%\n", soil.moisture);
  Serial.printf("[TX]   GPS      : %.6f, %.6f\n", latVal, lngVal);
  Serial.println("[TX] ────────────────────────────────\n");
}