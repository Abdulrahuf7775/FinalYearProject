#include <Arduino.h>
#include <DHT.h>
#include <TinyGPSPlus.h>

// ════════════════════════════════════════════════════════════
//  HARDWARE & CONFIGURATION
// ════════════════════════════════════════════════════════════
#define NODE_ID         "NODE_01"
#define LOG_INTERVAL_MS  180000

#define GPS_RX          14
#define GPS_TX          27
#define GPS_BAUD        9600
#define GPS_WAIT_MS     2000
#define GPS_LED_PIN     2

#define NPK_RX          34
#define NPK_TX          16
#define RS485_DE        21

#define DHT_PIN         17
#define DHT_TYPE        DHT11

#define LORA_RX         4
#define LORA_TX         32
#define LORA_MDO        22
#define LORA_BAUD       9600

#define MOISTURE_PIN    25

#define RAINY_MOISTURE_MIN  80.0f
#define RAINY_HUMIDITY_MIN  70.0f
#define RAINY_TEMP_MAX      25.0f

HardwareSerial gpsSerial(1);
HardwareSerial uart2(2);
DHT            dht(DHT_PIN, DHT_TYPE);
TinyGPSPlus    gps;

unsigned long lastLog   = 0;
unsigned long lastBlink = 0;
bool          ledState  = false;

// ════════════════════════════════════════════════════════════
//  CRC-16 Modbus (Polynomial 0xA001)
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
//  Real Moisture Sensor — GPIO 25 (AO)
// ════════════════════════════════════════════════════════════
int getMoisture() {
  int raw = analogRead(MOISTURE_PIN);
  int pct = (int)(((4095 - raw) / 4095.0f) * 100.0f);

  Serial.println("  ── Moisture (Real Sensor) ──────────");
  Serial.printf ("  Raw   : %d\n", raw);
  Serial.printf ("  Pct   : %d %%\n", pct);
  Serial.print  ("  State : ");
  if      (pct < 20)  Serial.println("DRY — Consider watering!");
  else if (pct < 50)  Serial.println("SLIGHTLY MOIST");
  else if (pct < 75)  Serial.println("MOIST — OPTIMAL");
  else                Serial.println("WET — Reduce watering!");
  Serial.println("  ────────────────────────────────────");

  return pct;
}

// ════════════════════════════════════════════════════════════
//  NPK — Sequential registers 0x001E, 0x001F, 0x0020
// ════════════════════════════════════════════════════════════
bool readNPK(int &n, int &p, int &k) {
  n = p = k = 0;

  uart2.begin(9600, SERIAL_8N1, NPK_RX, NPK_TX);
  delay(50);

  auto readRegister = [](uint16_t regAddress, int &value) -> bool {
    digitalWrite(RS485_DE, LOW);
    while (uart2.available()) uart2.read();

    uint8_t frame[8] = {0x01, 0x03,
                        (uint8_t)(regAddress >> 8), (uint8_t)(regAddress & 0xFF),
                        0x00, 0x01, 0, 0};
    uint16_t crc = calculateCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    digitalWrite(RS485_DE, HIGH);
    delay(2);
    uart2.write(frame, 8);
    uart2.flush();
    delay(5);
    digitalWrite(RS485_DE, LOW);

    uint8_t buf[10];
    int idx = 0;
    unsigned long start = millis();
    while (millis() - start < 200 && idx < 7) {
      if (uart2.available()) buf[idx++] = uart2.read();
    }

    if (idx >= 7) {
      uint16_t calcCRC = calculateCRC(buf, 5);
      uint16_t recvCRC = buf[5] | (buf[6] << 8);
      if (calcCRC == recvCRC && buf[1] == 0x03 && buf[2] == 0x02) {
        value = (buf[3] << 8) | buf[4];
        return true;
      }
    }
    return false;
  };

  bool okN = readRegister(0x001E, n); delay(50);
  bool okP = readRegister(0x001F, p); delay(50);
  bool okK = readRegister(0x0020, k);

  uart2.end();
  pinMode(NPK_TX, INPUT);
  delay(20);

  if (okN || okP || okK) {
    Serial.printf("  [NPK] OK: N:%d  P:%d  K:%d mg/kg\n", n, p, k);
    return true;
  } else {
    Serial.println("  [NPK] FAIL: No valid response from sensor");
    return false;
  }
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
            gps.date.year(),   gps.date.month(),  gps.date.day(),
            gps.time.hour(),   gps.time.minute(), gps.time.second());
    return String(buf);
  }
  unsigned long s = millis() / 1000;
  char buf[20];
  sprintf(buf, "UPTIME %02lu:%02lu:%02lu", s / 3600, (s % 3600) / 60, s % 60);
  return String(buf);
}

String classifySeason(float m, float h, float t) {
  return (m > RAINY_MOISTURE_MIN && h > RAINY_HUMIDITY_MIN && t < RAINY_TEMP_MAX)
         ? "Rainy" : "Dry";
}

// ════════════════════════════════════════════════════════════
//  LoRa TX (shares UART2)
// ════════════════════════════════════════════════════════════
bool sendLoRa(int mp, float temp, float hum,
              int n, int p, int k,
              float lat, float lng, float alt, String sats,
              String season) {

  String pkt = String(NODE_ID) + "," + mp + "," +
               String(temp, 1) + "," + String(hum, 1) + "," +
               n + "," + p + "," + k + "," +
               String(lat, 6) + "," + String(lng, 6) + "," +
               String(alt, 1) + "," + sats + "," + season;

  Serial.println("\n[LoRa] Sending: " + pkt);

  uart2.begin(LORA_BAUD, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(200);
  while (uart2.available()) uart2.read();

  digitalWrite(LORA_MDO, LOW);
  uart2.println(pkt);
  uart2.flush();
  delay(200);
  while (uart2.available()) uart2.read();

  uart2.end();
  delay(50);

  Serial.printf("[LoRa] Sent (%d bytes).\n", pkt.length() + 1);
  return true;
}

// ════════════════════════════════════════════════════════════
//  Print Readings
// ════════════════════════════════════════════════════════════
void printReadings(int moisturePct,
                   float temperature, float humidity, bool dhtOk,
                   int nitrogen, int phosphorus, int potassium, bool npkOk,
                   bool gpsOk, float lat, float lng, float alt, String sats,
                   String season) {

  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.printf( "║   %s — SENSOR READINGS           ║\n", NODE_ID);
  Serial.println( "╚══════════════════════════════════════════╝");
  Serial.println( "  Timestamp   : " + getTimestamp());

  Serial.println("  ────────────────────────────────────────");
  Serial.printf ("  Moisture    : %d %%\n", moisturePct);
  Serial.print  ("  Soil Status : ");
  if      (moisturePct < 20) Serial.println("DRY  — Consider watering!");
  else if (moisturePct < 50) Serial.println("SLIGHTLY MOIST");
  else if (moisturePct < 75) Serial.println("MOIST — OPTIMAL");
  else                        Serial.println("WET  — Reduce watering!");

  Serial.println("  ────────────────────────────────────────");
  if (dhtOk) {
    Serial.printf("  Temperature : %.1f C\n",  temperature);
    Serial.printf("  Humidity    : %.1f %%\n", humidity);
  } else {
    Serial.println("  Temperature : ERROR — DHT11 not responding");
    Serial.println("  Humidity    : ERROR — DHT11 not responding");
  }

  Serial.println("  ────────────────────────────────────────");
  if (npkOk) {
    Serial.printf("  Nitrogen  N : %d mg/kg\n", nitrogen);
    Serial.printf("  Phosphorus P: %d mg/kg\n", phosphorus);
    Serial.printf("  Potassium K : %d mg/kg\n", potassium);
  } else {
    Serial.println("  NPK         : ERROR — No valid response from sensor");
  }

  Serial.println("  ────────────────────────────────────────");
  if (gpsOk) {
    Serial.printf("  Latitude    : %.6f\n", lat);
    Serial.printf("  Longitude   : %.6f\n", lng);
    Serial.printf("  Altitude    : %.1f m\n", alt);
    Serial.printf("  Satellites  : %s\n",    sats.c_str());
    if (gps.time.isValid())
      Serial.printf("  UTC Time    : %02d:%02d:%02d\n",
                    gps.time.hour(), gps.time.minute(), gps.time.second());
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
  pinMode(LORA_TX,      OUTPUT); digitalWrite(LORA_TX,     HIGH);
  pinMode(LORA_MDO,     OUTPUT); digitalWrite(LORA_MDO,    LOW);
  pinMode(NPK_TX,       INPUT);
  pinMode(RS485_DE,     OUTPUT); digitalWrite(RS485_DE,    LOW);
  pinMode(GPS_LED_PIN,  OUTPUT); digitalWrite(GPS_LED_PIN, LOW);
  pinMode(MOISTURE_PIN, INPUT);

  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==========================================");
  Serial.printf( "   %s — LoRa Agri Node\n", NODE_ID);
  Serial.println("   Sensors → LoRa TX → Gateway");
  Serial.println("   NPK: Sequential 0x001E, 0x001F, 0x0020 @ 9600");
  Serial.println("   Moisture: Real Sensor on GPIO 25");
  Serial.println("   GPS: Float values (6 decimal places)");
  Serial.println("==========================================\n");

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[INIT] GPS  UART1 (GPIO 14/27).");

  pinMode(DHT_PIN, INPUT_PULLUP);
  delay(100);
  dht.begin();
  Serial.println("[INIT] DHT11 (GPIO 17).");

  Serial.println("[INIT] Moisture Sensor on GPIO 25 (AO).");
  Serial.printf ("[INIT] Shared UART2: NPK (9600) ↔ LoRa (9600)\n");
  Serial.println("==========================================");
  Serial.println("[INIT] Ready! Logging every 10s.\n");

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

  // 1. Moisture
  int moisturePct = getMoisture();

  // 2. NPK
  int  nitrogen = 0, phosphorus = 0, potassium = 0;
  bool npkOk    = readNPK(nitrogen, phosphorus, potassium);

  // 3. DHT
  float temperature = dht.readTemperature();
  float humidity    = dht.readHumidity();
  bool  dhtOk       = !isnan(temperature) && !isnan(humidity);
  if (!dhtOk) { temperature = 0.0f; humidity = 0.0f; }

  // 4. GPS
  bool   gpsOk   = readGPS();
  float  latVal  = gpsOk ? (float)gps.location.lat()    : 0.0f;
  float  lngVal  = gpsOk ? (float)gps.location.lng()    : 0.0f;
  float  altVal  = (gpsOk && gps.altitude.isValid())    ? (float)gps.altitude.meters() : 0.0f;
  String satsStr = (gpsOk && gps.satellites.isValid())  ? String(gps.satellites.value()) : "ERR";

  // 5. Season
  String season = classifySeason((float)moisturePct, humidity, temperature);

  // 6. Print
  printReadings(moisturePct,
                temperature, humidity,   dhtOk,
                nitrogen,    phosphorus, potassium, npkOk,
                gpsOk,       latVal,     lngVal,    altVal, satsStr,
                season);

  // 7. LoRa
  Serial.println("[TX] Sending via LoRa...");
  bool loraSent = sendLoRa(moisturePct, temperature, humidity,
                           nitrogen,    phosphorus,  potassium,
                           latVal,      lngVal,      altVal,
                           satsStr,     season);

  Serial.println("\n[TX] ── Summary ─────────────────────");
  Serial.printf("[TX]   LoRa     : %s\n", loraSent ? "SENT ✓" : "FAILED ✗");
  Serial.printf("[TX]   NPK      : %s\n", npkOk    ? "OK ✓"   : "ERROR ✗");
  Serial.printf("[TX]   Moisture : %d %%\n", moisturePct);
  Serial.printf("[TX]   GPS      : %.6f, %.6f\n", latVal, lngVal);
  Serial.println("[TX] ────────────────────────────────\n");
}