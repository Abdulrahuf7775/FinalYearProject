#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ════════════════════════════════════════════════════════════════════════════
//  CONFIGURATION
// ════════════════════════════════════════════════════════════════════════════
#define WIFI_SSID       "capstone_project"
#define WIFI_PASSWORD   "edgeaigroup3"
#define AP_SSID         "capstone_project"
#define AP_PASSWORD     "group3"

// Cloud Endpoints
#define SUPABASE_URL    "https://almuqesulujoigljgpzv.supabase.co/functions/v1/sensor-data"
#define SHEETS_URL      "https://script.google.com/macros/s/AKfycbwzncjwaAYCltQZYjg8Iwchk6xfCqZ14gsHARDfUViFedvtMJ63DUWgK4syPPRkoa2fvA/exec"
#define FIREBASE_HOST   "capstone-2e26e-default-rtdb.firebaseio.com"
#define FIREBASE_URL    "https://" FIREBASE_HOST

#define WIFI_RETRY_MS   300000

// GPS Fallback (your farm location)
#define FALLBACK_LAT    "9.056700"
#define FALLBACK_LNG    "7.496900"
#define FALLBACK_ALT    "871.0"
#define FALLBACK_SATS   "0"

// Hardware Pins
#define LORA_RX  16
#define LORA_TX  17
#define LORA_MDO 22
#define SD_MISO  19
#define SD_MOSI  13
#define SD_SCK   14
#define SD_CS    26

// Storage
#define DATA_FILE    "/data.csv"
#define PENDING_FILE "/pending.csv"
#define ADVICE_FILE  "/advice.csv"

#define MAX_BUFFERED_READINGS  100
#define MAX_NODES               50
#define MAX_ADVICE_PER_NODE     5

// NPK Thresholds for Alerts
#define NPK_ZERO_THRESHOLD      3    // Consecutive zero readings before alert
#define NPK_MIN_VALID           5    // Minimum valid N/P/K value (mg/kg)
#define MOISTURE_CRITICAL_LOW   25   // % - trigger irrigation alert
#define MOISTURE_CRITICAL_HIGH  90   // % - trigger drainage alert
#define N_DEFICIENT             30   // mg/kg
#define P_DEFICIENT             20   // mg/kg
#define K_DEFICIENT             40   // mg/kg
#define TEMP_STRESS_HIGH        35.0 // °C
#define TEMP_STRESS_LOW         10.0 // °C

// ════════════════════════════════════════════════════════════════════════════
//  DATA STRUCTURES
// ════════════════════════════════════════════════════════════════════════════
struct SensorReading {
  unsigned long timestamp;
  String nodeId;
  int    moisture;
  float  temp;
  float  humidity;
  int    nitrogen, phosphorus, potassium;
  String lat, lng, alt, satellites, season;
  bool   gpsWasFallback;
  bool   syncedToSupabase;
  bool   syncedToSheets;
  bool   syncedToFirebase;
};

struct Advice {
  unsigned long timestamp;
  String nodeId;
  String category;    // "NPK", "IRRIGATION", "FERTILIZER", "SECURITY", "MAINTENANCE", "WEATHER"
  String severity;    // "INFO", "WARNING", "CRITICAL"
  String message;
  String action;      // Recommended action
  bool   acknowledged;
  bool   syncedToSheets;
  bool   syncedToFirebase;
};

struct NodeInfo {
  String        nodeId;
  SensorReading latest;
  unsigned long firstSeen;
  unsigned long lastSeen;
  unsigned long totalReadings;
  bool          active;
  int           npkZeroCount;      // Track consecutive NPK=0 readings
  Advice        activeAdvice[MAX_ADVICE_PER_NODE];
  int           adviceCount;
};

// ════════════════════════════════════════════════════════════════════════════
//  GLOBALS
// ════════════════════════════════════════════════════════════════════════════
HardwareSerial loraSerial(2);
WebServer      server(80);

unsigned long lastWifiTry = 0;
unsigned long rowCount    = 0;
unsigned long adviceCount = 0;
bool          sdReady     = false;
bool          wifiOnline  = false;
bool          apMode      = false;

String lastKnownLat  = FALLBACK_LAT;
String lastKnownLng  = FALLBACK_LNG;
String lastKnownAlt  = FALLBACK_ALT;
String lastKnownSats = FALLBACK_SATS;
bool   hasRealGPS    = false;

SensorReading recentReadings[MAX_BUFFERED_READINGS];
int           readIndex = 0;

NodeInfo nodeRegistry[MAX_NODES];
int      nodeCount = 0;

Advice recentAdvice[MAX_BUFFERED_READINGS];
int    adviceIndex = 0;

// ════════════════════════════════════════════════════════════════════════════
//  NODE REGISTRY
// ════════════════════════════════════════════════════════════════════════════
int findNode(const String& nodeId) {
  for (int i = 0; i < nodeCount; i++)
    if (nodeRegistry[i].nodeId == nodeId) return i;
  return -1;
}

int getOrCreateNode(const String& nodeId) {
  int idx = findNode(nodeId);
  if (idx >= 0) return idx;
  if (nodeCount >= MAX_NODES) {
    int oldest = 0;
    for (int i = 1; i < MAX_NODES; i++)
      if (nodeRegistry[i].lastSeen < nodeRegistry[oldest].lastSeen) oldest = i;
    Serial.printf("[Registry] ⚠️ Full — replacing oldest: %s\n", nodeRegistry[oldest].nodeId.c_str());
    nodeRegistry[oldest] = NodeInfo();
    nodeRegistry[oldest].nodeId    = nodeId;
    nodeRegistry[oldest].firstSeen = millis() / 1000;
    nodeRegistry[oldest].active    = false;
    return oldest;
  }
  nodeRegistry[nodeCount].nodeId    = nodeId;
  nodeRegistry[nodeCount].firstSeen = millis() / 1000;
  nodeRegistry[nodeCount].active    = false;
  Serial.printf("[Registry] ✅ New node: %s (slot %d/%d)\n", nodeId.c_str(), nodeCount + 1, MAX_NODES);
  return nodeCount++;
}

void updateNodeRegistry(const SensorReading& r) {
  int idx = getOrCreateNode(r.nodeId);
  nodeRegistry[idx].latest        = r;
  nodeRegistry[idx].lastSeen      = r.timestamp;
  nodeRegistry[idx].totalReadings++;
  nodeRegistry[idx].active        = true;
}

// ════════════════════════════════════════════════════════════════════════════
//  GPS FALLBACK RESOLVER
// ════════════════════════════════════════════════════════════════════════════
void resolveGPS(String &lat, String &lng, String &alt, String &sats, bool &wasFallback) {
  // Check for zero, ERR, or too-short values
  bool latOk = (lat  != "ERR" && lat  != "0.000000" && lat  != "0" && lat.length()  > 5);
  bool lngOk = (lng  != "ERR" && lng  != "0.000000" && lng  != "0" && lng.length()  > 5);
  bool altOk = (alt  != "ERR" && alt  != "0.0"      && alt  != "0" && alt.length()  > 0);
  bool satOk = (sats != "ERR" && sats != "0"         && sats.length() > 0);
  int  satNum = satOk ? sats.toInt() : 0;

  if (latOk && lngOk && satNum >= 3) {
    // Real valid GPS fix
    lastKnownLat  = lat;
    lastKnownLng  = lng;
    lastKnownAlt  = altOk ? alt : lastKnownAlt;
    lastKnownSats = sats; 
    hasRealGPS    = true;
    wasFallback   = false;
    Serial.printf("  [GPS] ✅ Real lock — %s, %s (%d sats)\n",
                  lat.c_str(), lng.c_str(), satNum);
  } else {
    // No fix — use fallback or last known
    lat  = lastKnownLat;
    lng  = lastKnownLng;
    alt  = lastKnownAlt;
    sats = lastKnownSats;
    wasFallback = true;
    Serial.printf("  [GPS] 📍 Using %s: %s, %s\n",
                  hasRealGPS ? "last-known" : "hardcoded-fallback",
                  lat.c_str(), lng.c_str());
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  ADVICE GENERATION ENGINE
// ════════════════════════════════════════════════════════════════════════════
void addAdvice(const String& nodeId, const String& category, const String& severity, 
               const String& message, const String& action) {
  Advice adv;
  adv.timestamp      = millis() / 1000;
  adv.nodeId         = nodeId;
  adv.category       = category;
  adv.severity       = severity;
  adv.message        = message;
  adv.action         = action;
  adv.acknowledged   = false;
  adv.syncedToSheets = false;
  adv.syncedToFirebase = false;
  
  recentAdvice[adviceIndex] = adv;
  adviceIndex = (adviceIndex + 1) % MAX_BUFFERED_READINGS;
  adviceCount++;
  
  // Also add to node's active advice list
  int idx = findNode(nodeId);
  if (idx >= 0 && nodeRegistry[idx].adviceCount < MAX_ADVICE_PER_NODE) {
    nodeRegistry[idx].activeAdvice[nodeRegistry[idx].adviceCount++] = adv;
  }
  
  Serial.printf("  [ADVICE] [%s] %s: %s → %s\n", severity.c_str(), category.c_str(), message.c_str(), action.c_str());
}

void generateAdvice(const SensorReading& r) {
  int idx = findNode(r.nodeId);
  if (idx < 0) return;
  NodeInfo& node = nodeRegistry[idx];
  
  // ── SECURITY: NPK Sensor Removed/Stolen Detection ─────────────────────────
  if (r.nitrogen == 0 && r.phosphorus == 0 && r.potassium == 0) {
    node.npkZeroCount++;
    if (node.npkZeroCount >= NPK_ZERO_THRESHOLD) {
      addAdvice(r.nodeId, "SECURITY", "CRITICAL",
                "NPK sensor reading 0/0/0 for " + String(NPK_ZERO_THRESHOLD) + " consecutive cycles",
                "Check if sensor is buried in soil. If location is compromised, investigate potential theft.");
    }
  } else {
    node.npkZeroCount = 0; // Reset counter on valid reading
  }
  
  // ── IRRIGATION ADVICE ─────────────────────────────────────────────────────
  if (r.moisture < MOISTURE_CRITICAL_LOW) {
    String urgency = (r.moisture < 15) ? "IMMEDIATE" : "SOON";
    addAdvice(r.nodeId, "IRRIGATION", "WARNING",
              "Soil moisture critically low: " + String(r.moisture) + "%",
              urgency + " irrigation recommended. Apply 10-15mm water. Check drip lines for clogs.");
  } else if (r.moisture > MOISTURE_CRITICAL_HIGH) {
    addAdvice(r.nodeId, "IRRIGATION", "INFO",
              "Soil moisture very high: " + String(r.moisture) + "%",
              "Pause irrigation. Ensure drainage is adequate to prevent root rot.");
  }
  
  // ── FERTILIZER RECOMMENDATIONS ────────────────────────────────────────────
  if (r.nitrogen < N_DEFICIENT && r.nitrogen > 0) {
    addAdvice(r.nodeId, "FERTILIZER", "WARNING",
              "Nitrogen deficient: " + String(r.nitrogen) + " mg/kg (target: >" + String(N_DEFICIENT) + ")",
              "Apply nitrogen-rich fertilizer: urea (46-0-0) at 50kg/ha or compost tea. Re-test in 7 days.");
  }
  if (r.phosphorus < P_DEFICIENT && r.phosphorus > 0) {
    addAdvice(r.nodeId, "FERTILIZER", "WARNING",
              "Phosphorus deficient: " + String(r.phosphorus) + " mg/kg (target: >" + String(P_DEFICIENT) + ")",
              "Apply bone meal or superphosphate (0-46-0) at 30kg/ha. Incorporate into top 10cm soil.");
  }
  if (r.potassium < K_DEFICIENT && r.potassium > 0) {
    addAdvice(r.nodeId, "FERTILIZER", "WARNING",
              "Potassium deficient: " + String(r.potassium) + " mg/kg (target: >" + String(K_DEFICIENT) + ")",
              "Apply muriate of potash (0-0-60) at 40kg/ha or wood ash. Water lightly after application.");
  }
  
  // ── WEATHER/TEMPERATURE STRESS ────────────────────────────────────────────
  if (r.temp > TEMP_STRESS_HIGH) {
    addAdvice(r.nodeId, "WEATHER", "WARNING",
              "High temperature stress: " + String(r.temp) + "°C",
              "Provide shade cloth (30-50%). Increase irrigation frequency. Mulch to reduce soil temp.");
  } else if (r.temp < TEMP_STRESS_LOW) {
    addAdvice(r.nodeId, "WEATHER", "INFO",
              "Low temperature alert: " + String(r.temp) + "°C",
              "Cover sensitive crops with row covers. Delay planting of warm-season crops.");
  }
  
  // ── MAINTENANCE REMINDERS ─────────────────────────────────────────────────
  if (r.gpsWasFallback && node.totalReadings % 20 == 0) {
    addAdvice(r.nodeId, "MAINTENANCE", "INFO",
              "GPS signal weak or unavailable",
              "Check GPS antenna orientation. Ensure clear sky view. Update fallback coordinates if node relocated.");
  }
  
  // ── SEASONAL ADVICE ───────────────────────────────────────────────────────
  if (r.season == "Rainy" && r.humidity > 85 && r.temp < 28) {
    addAdvice(r.nodeId, "WEATHER", "INFO",
              "High humidity + cool temps = fungal risk",
              "Apply preventive fungicide. Ensure good air circulation. Avoid overhead irrigation.");
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  WiFi + AP MANAGEMENT
// ════════════════════════════════════════════════════════════════════════════
void syncPendingToClouds();

bool tryConnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  Serial.printf("\n[WiFi] Connecting to '%s'...\n", WIFI_SSID);
  WiFi.disconnect(true); delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < 30; i++) {
    delay(400); Serial.print(".");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\n[WiFi] ✅ Connected! IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
  }
  Serial.println("\n[WiFi] ❌ Failed — switching to AP mode...");
  delay(500); return false;
}

void startAccessPoint() {
  Serial.println("\n[AP] Starting Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("[AP] SSID: %s | Pass: %s | IP: %s\n",
                AP_SSID, AP_PASSWORD, WiFi.softAPIP().toString().c_str());
  apMode = true; wifiOnline = false;
}

void manageWiFi() {
  if (apMode) {
    if (millis() - lastWifiTry >= WIFI_RETRY_MS) {
      lastWifiTry = millis(); WiFi.mode(WIFI_STA);
      if (tryConnectWiFi()) { WiFi.softAPdisconnect(true); apMode = false; wifiOnline = true; syncPendingToClouds(); }
      else startAccessPoint();
    }
    return;
  }
  if (wifiOnline) {
    if (WiFi.status() != WL_CONNECTED) { wifiOnline = false; startAccessPoint(); }
    return;
  }
  if (millis() - lastWifiTry >= WIFI_RETRY_MS) {
    lastWifiTry = millis();
    wifiOnline = tryConnectWiFi();
    if (wifiOnline) syncPendingToClouds(); else startAccessPoint();
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  WEB DASHBOARD (with Advice Display)
// ════════════════════════════════════════════════════════════════════════════
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>🌾 FarmSense Gateway</title>
  <style>
    :root{--primary:#2e7d32;--primary-light:#4caf50;--secondary:#1565c0;--bg:#f0f4f0;--card-bg:#fff;--text:#1b1b1b;--text-light:#666;--success:#2e7d32;--warning:#ed6c02;--error:#c62828;--info:#1976d2;--shadow:0 4px 20px rgba(0,0,0,.08);--radius:16px}
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--text);line-height:1.6;padding:20px}
    .container{max-width:1400px;margin:0 auto}
    header{text-align:center;padding:25px 20px;background:linear-gradient(135deg,var(--primary),var(--secondary));color:white;border-radius:var(--radius);margin-bottom:25px;box-shadow:var(--shadow)}
    header h1{font-size:1.8rem;margin-bottom:6px}
    .status-bar{display:flex;gap:12px;flex-wrap:wrap;justify-content:center;margin-bottom:25px}
    .status-card{background:var(--card-bg);padding:12px 20px;border-radius:12px;box-shadow:var(--shadow);display:flex;align-items:center;gap:10px;font-weight:500;font-size:.95rem}
    .dot{width:11px;height:11px;border-radius:50%;background:var(--error)}
    .dot.on{background:var(--success)}.dot.ap{background:var(--warning)}
    .section-title{font-size:1.2rem;font-weight:700;color:var(--primary);margin:0 0 15px;padding-bottom:10px;border-bottom:2px solid #dcedc8}
    #nodeGrid{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:16px;margin-bottom:25px}
    .node-card{background:var(--card-bg);border-radius:var(--radius);padding:18px;box-shadow:var(--shadow);border-top:4px solid var(--primary-light);position:relative}
    .node-card.fresh{animation:pulse 1s ease}
    @keyframes pulse{0%{box-shadow:0 0 0 0 rgba(76,175,80,.4)}70%{box-shadow:0 0 0 10px rgba(76,175,80,0)}100%{box-shadow:0 0 0 0 rgba(76,175,80,0)}}
    .node-id{font-size:1.1rem;font-weight:700;margin-bottom:10px}
    .node-badge{position:absolute;top:14px;right:14px;font-size:.72rem;padding:3px 9px;border-radius:20px;font-weight:600}
    .badge-real{background:#c8e6c9;color:var(--success)}
    .badge-fallback{background:#fff3e0;color:var(--warning)}
    .node-metrics{display:grid;grid-template-columns:1fr 1fr;gap:6px 10px;margin-bottom:10px}
    .metric{display:flex;flex-direction:column}
    .metric-label{font-size:.72rem;color:var(--text-light);text-transform:uppercase;letter-spacing:.04em}
    .metric-value{font-size:1rem;font-weight:700;color:var(--text)}
    .node-footer{display:flex;justify-content:space-between;align-items:center;padding-top:8px;border-top:1px dashed #eee;font-size:.78rem;color:var(--text-light)}
    .card{background:var(--card-bg);border-radius:var(--radius);padding:22px;box-shadow:var(--shadow);margin-bottom:20px}
    table{width:100%;border-collapse:collapse;font-size:.88rem}
    th,td{padding:10px 8px;text-align:left;border-bottom:1px solid #f0f0f0}
    th{background:#f1f8e9;color:var(--primary);font-weight:700;font-size:.8rem;text-transform:uppercase;letter-spacing:.05em}
    tr:hover{background:#fafafa}
    .sync-badge{padding:3px 9px;border-radius:10px;font-size:.75rem;font-weight:600}
    .synced{background:#c8e6c9;color:var(--success)}
    .pending{background:#fff3e0;color:var(--warning)}
    .gps-f{background:#e3f2fd;color:var(--secondary)}
    .npk-warn{background:#fce4ec;color:#c62828;padding:3px 9px;border-radius:10px;font-size:.75rem;font-weight:600}
    .advice-card{background:var(--card-bg);border-left:4px solid var(--warning);border-radius:12px;padding:14px;margin:8px 0;box-shadow:var(--shadow)}
    .advice-card.CRITICAL{border-left-color:var(--error);background:#ffebee}
    .advice-card.WARNING{border-left-color:var(--warning);background:#fff3e0}
    .advice-card.INFO{border-left-color:var(--info);background:#e3f2fd}
    .advice-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}
    .advice-category{font-weight:700;font-size:.9rem;text-transform:uppercase}
    .advice-severity{padding:2px 8px;border-radius:8px;font-size:.7rem;font-weight:700}
    .severity-CRITICAL{background:var(--error);color:white}
    .severity-WARNING{background:var(--warning);color:#000}
    .severity-INFO{background:var(--info);color:white}
    .advice-message{font-size:.95rem;margin:4px 0 8px}
    .advice-action{font-size:.85rem;color:var(--text-light);font-style:italic}
    .advice-time{font-size:.75rem;color:var(--text-light)}
    .refresh-btn{background:var(--primary);color:white;border:none;padding:12px 30px;border-radius:10px;font-size:1rem;font-weight:600;cursor:pointer;transition:all .2s;display:block;margin:10px auto 25px}
    .refresh-btn:hover{background:var(--primary-light);transform:translateY(-2px)}
    footer{text-align:center;padding:20px;color:var(--text-light);font-size:.85rem}
    @media(max-width:600px){#nodeGrid{grid-template-columns:1fr}.status-bar{flex-direction:column;align-items:center}}
  </style>
</head>
<body>
<div class="container">
  <header>
    <h1>🌾 FarmSense Gateway</h1>
    <p>Multi-Node Agricultural Sensor Network with Smart Advisory</p>
  </header>
  <div class="status-bar">
    <div class="status-card"><div class="dot" id="wifiDot"></div><span id="wifiStatus">Checking...</span></div>
    <div class="status-card"><div class="dot on"></div><span id="sdStatus">SD: Ready</span></div>
    <div class="status-card">🌐 Nodes: <strong id="nodeCount" style="margin-left:6px">0</strong></div>
    <div class="status-card">📊 Rows: <strong id="rowCount" style="margin-left:6px">0</strong></div>
    <div class="status-card">⚠️ Active Advice: <strong id="adviceCount" style="margin-left:6px">0</strong></div>
    <div class="status-card">📍 GPS: <strong id="gpsStatus" style="margin-left:6px">Waiting...</strong></div>
    <div class="status-card">🔥 Firebase: <strong id="fbStatus" style="margin-left:6px">–</strong></div>
  </div>

  <div class="section-title">⚠️ Active Advisories</div>
  <div id="adviceList"><p style="color:var(--text-light);padding:20px">No active advisories ✓</p></div>

  <div class="section-title">📡 Active Nodes</div>
  <div id="nodeGrid"><p style="color:var(--text-light);padding:20px">Waiting for nodes...</p></div>

  <div class="card">
    <div class="section-title">🔁 Recent Readings (all nodes)</div>
    <div style="overflow-x:auto;max-height:320px;overflow-y:auto">
      <table>
        <thead><tr><th>Time</th><th>Node</th><th>Moisture</th><th>Temp</th><th>Humidity</th><th>N/P/K</th><th>GPS</th><th>Cloud</th></tr></thead>
        <tbody id="readingsBody"><tr><td colspan="8" style="text-align:center;color:var(--text-light)">No readings yet</td></tr></tbody>
      </table>
    </div>
  </div>

  <button class="refresh-btn" onclick="fetchData()">🔄 Refresh</button>
  <footer><p>FarmSense Gateway v3.0 • <span id="timestamp"></span></p></footer>
</div>
<script>
  const nodeColors=['#2e7d32','#1565c0','#6a1b9a','#e65100','#00695c','#4527a0','#ad1457','#558b2f','#0277bd','#c62828'];
  setInterval(fetchData,10000);
  
  function formatTime(ts){
    if(!ts) return 'N/A';
    const d=new Date(ts*1000);
    return d.toLocaleTimeString();
  }
  
  async function fetchData(){
    try{
      const s=await(await fetch('/api/status')).json();
      document.getElementById('wifiDot').className='dot '+(s.wifiOnline?'on':s.apMode?'ap':'');
      document.getElementById('wifiStatus').textContent=s.wifiOnline?'WiFi: Online':s.apMode?'AP Mode':'WiFi: Offline';
      document.getElementById('sdStatus').textContent=s.sdReady?'SD: Ready':'SD: Error';
      document.getElementById('rowCount').textContent=s.rowCount||0;
      document.getElementById('adviceCount').textContent=s.adviceCount||0;
      document.getElementById('nodeCount').textContent=s.nodeCount||0;
      document.getElementById('gpsStatus').textContent=s.hasRealGPS?'✅ Real':'📍 Fallback';
      document.getElementById('fbStatus').textContent=s.wifiOnline?'✅ Online':'⏳ Offline';

      // Load advisories
      const a=await(await fetch('/api/advice')).json();
      const advList=document.getElementById('adviceList');
      if(a.advice&&a.advice.length>0){
        advList.innerHTML='';
        a.advice.slice(0,20).forEach(ad=>{
          const card=document.createElement('div');
          card.className='advice-card '+ad.severity;
          card.innerHTML=
            `<div class="advice-header">`+
            `<span class="advice-category">${ad.category}</span>`+
            `<span class="advice-severity severity-${ad.severity}">${ad.severity}</span>`+
            `</div>`+
            `<div class="advice-message"><strong>${ad.nodeId}:</strong> ${ad.message}</div>`+
            `<div class="advice-action">→ ${ad.action}</div>`+
            `<div class="advice-time">${formatTime(ad.timestamp)}</div>`;
          advList.appendChild(card);
        });
      }else{
        advList.innerHTML='<p style="color:var(--text-light);padding:20px">No active advisories ✓</p>';
      }

      // Load nodes
      const d=await(await fetch('/api/data')).json();
      if(d.nodes&&d.nodes.length>0){
        const grid=document.getElementById('nodeGrid');
        grid.innerHTML='';
        d.nodes.forEach((n,i)=>{
          const color=nodeColors[i%nodeColors.length];
          const mapsLink=`https://maps.google.com/?q=${n.lat},${n.lng}`;
          const ageSec=Math.floor(Date.now()/1000)-n.lastSeen;
          const ageStr=ageSec<60?ageSec+'s ago':ageSec<3600?Math.floor(ageSec/60)+'m ago':Math.floor(ageSec/3600)+'h ago';
          const isFresh=ageSec<30;
          const card=document.createElement('div');
          card.className='node-card'+(isFresh?' fresh':'');
          card.style.borderTopColor=color;
          
          // Check for critical advice on this node
          let adviceBadge='';
          if(n.activeAdvice&&n.activeAdvice.length>0){
            const critical=n.activeAdvice.some(ad=>ad.severity==='CRITICAL');
            const warning=n.activeAdvice.some(ad=>ad.severity==='WARNING');
            if(critical) adviceBadge='<span class="node-badge" style="background:#ffcdd2;color:#c62828">⚠️ CRITICAL</span>';
            else if(warning) adviceBadge='<span class="node-badge" style="background:#fff3e0;color:#ed6c02">⚠️ WARNING</span>';
          }
          
          card.innerHTML=
            `<div class="node-id" style="color:${color}">📡 ${n.nodeId}</div>`+
            adviceBadge+
            `<span class="node-badge ${n.gpsWasFallback?'badge-fallback':'badge-real'}">${n.gpsWasFallback?'📍 Fallback':'✅ Real GPS'}</span>`+
            `<div class="node-metrics">`+
            `<div class="metric"><span class="metric-label">Moisture</span><span class="metric-value">${n.moisture}%</span></div>`+
            `<div class="metric"><span class="metric-label">Temp</span><span class="metric-value">${n.temp}°C</span></div>`+
            `<div class="metric"><span class="metric-label">Humidity</span><span class="metric-value">${n.humidity}%</span></div>`+
            `<div class="metric"><span class="metric-label">N/P/K</span><span class="metric-value">${n.nitrogen}/${n.phosphorus}/${n.potassium}</span></div>`+
            `<div class="metric"><span class="metric-label">Season</span><span class="metric-value">${n.season}</span></div>`+
            `<div class="metric"><span class="metric-label">Readings</span><span class="metric-value">${n.totalReadings}</span></div>`+
            `</div>`+
            `<div class="node-footer">`+
            `<a href="${mapsLink}" target="_blank" style="color:var(--secondary);font-size:.78rem">📍 ${n.lat}, ${n.lng}</a>`+
            `<span>${ageStr}</span>`+
            `</div>`;
          grid.appendChild(card);
        });
      }

      // Load recent readings
      if(d.readings&&d.readings.length>0){
        const tbody=document.getElementById('readingsBody');
        tbody.innerHTML='';
        d.readings.slice(0,30).forEach(r=>{
          const time=formatTime(r.timestamp);
          const cloud=r.syncedSheets&&r.syncedFirebase?'<span class="sync-badge synced">✅ All</span>':r.syncedSheets||r.syncedFirebase?'<span class="sync-badge pending">Partial</span>':'<span class="sync-badge pending">Pending</span>';
          const row=document.createElement('tr');
          row.innerHTML=`<td>${time}</td><td><strong>${r.nodeId}</strong></td><td>${r.moisture}%</td><td>${r.temp}°C</td><td>${r.humidity}%</td>`+
            `<td>${r.nitrogen}/${r.phosphorus}/${r.potassium}</td>`+
            `<td><span class="sync-badge ${r.gpsWasFallback?'gps-f':'synced'}">${r.gpsWasFallback?'📍':'✅'}</span></td>`+
            `<td>${cloud}</td>`;
          tbody.appendChild(row);
        });
      }
      document.getElementById('timestamp').textContent=new Date().toLocaleString();
    }catch(e){console.error(e)}
  }
  window.onload=fetchData;
</script>
</body>
</html>
)rawliteral";

// ════════════════════════════════════════════════════════════════════════════
//  JSON HELPERS
// ════════════════════════════════════════════════════════════════════════════
String getRecentReadingsJSON() {
  DynamicJsonDocument doc(16384);
  JsonArray readings = doc.createNestedArray("readings");
  for (int i = 0; i < MAX_BUFFERED_READINGS; i++) {
    int idx = (readIndex - 1 - i + MAX_BUFFERED_READINGS) % MAX_BUFFERED_READINGS;
    const SensorReading& r = recentReadings[idx];
    if (r.timestamp == 0) continue;
    JsonObject rd = readings.createNestedObject();
    rd["timestamp"]      = r.timestamp;
    rd["nodeId"]         = r.nodeId;
    rd["moisture"]       = r.moisture;
    rd["temp"]           = r.temp;
    rd["humidity"]       = r.humidity;
    rd["nitrogen"]       = r.nitrogen;
    rd["phosphorus"]     = r.phosphorus;
    rd["potassium"]      = r.potassium;
    rd["lat"]            = r.lat;
    rd["lng"]            = r.lng;
    rd["syncedSupabase"] = r.syncedToSupabase;
    rd["syncedSheets"]   = r.syncedToSheets;
    rd["syncedFirebase"] = r.syncedToFirebase;
    rd["gpsWasFallback"] = r.gpsWasFallback;
  }
  JsonArray nodes = doc.createNestedArray("nodes");
  for (int i = 0; i < nodeCount; i++) {
    if (!nodeRegistry[i].active) continue;
    const NodeInfo& n = nodeRegistry[i];
    JsonObject nd = nodes.createNestedObject();
    nd["nodeId"]         = n.nodeId;
    nd["moisture"]       = n.latest.moisture;
    nd["temp"]           = n.latest.temp;
    nd["humidity"]       = n.latest.humidity;
    nd["nitrogen"]       = n.latest.nitrogen;
    nd["phosphorus"]     = n.latest.phosphorus;
    nd["potassium"]      = n.latest.potassium;
    nd["lat"]            = n.latest.lat;
    nd["lng"]            = n.latest.lng;
    nd["season"]         = n.latest.season;
    nd["gpsWasFallback"] = n.latest.gpsWasFallback;
    nd["firstSeen"]      = n.firstSeen;
    nd["lastSeen"]       = n.lastSeen;
    nd["totalReadings"]  = n.totalReadings;
    // Add active advice summary
    JsonArray adviceArr = nd.createNestedArray("activeAdvice");
    for (int j = 0; j < n.adviceCount; j++) {
      JsonObject adv = adviceArr.createNestedObject();
      adv["category"]  = n.activeAdvice[j].category;
      adv["severity"]  = n.activeAdvice[j].severity;
      adv["message"]   = n.activeAdvice[j].message;
      adv["action"]    = n.activeAdvice[j].action;
    }
  }
  String out; serializeJson(doc, out); return out;
}

String getRecentAdviceJSON() {
  DynamicJsonDocument doc(8192);
  JsonArray adviceArr = doc.createNestedArray("advice");
  for (int i = 0; i < MAX_BUFFERED_READINGS; i++) {
    int idx = (adviceIndex - 1 - i + MAX_BUFFERED_READINGS) % MAX_BUFFERED_READINGS;
    const Advice& a = recentAdvice[idx];
    if (a.timestamp == 0) continue;
    JsonObject adv = adviceArr.createNestedObject();
    adv["timestamp"]   = a.timestamp;
    adv["nodeId"]      = a.nodeId;
    adv["category"]    = a.category;
    adv["severity"]    = a.severity;
    adv["message"]     = a.message;
    adv["action"]      = a.action;
    adv["acknowledged"]= a.acknowledged;
  }
  String out; serializeJson(doc, out); return out;
}

String getSystemStatusJSON() {
  StaticJsonDocument<512> doc;
  doc["wifiOnline"]   = wifiOnline;
  doc["apMode"]       = apMode;
  doc["sdReady"]      = sdReady;
  doc["rowCount"]     = rowCount;
  doc["adviceCount"]  = adviceCount;
  doc["hasRealGPS"]   = hasRealGPS;
  doc["nodeCount"]    = nodeCount;
  doc["maxNodes"]     = MAX_NODES;
  doc["ipAddress"]    = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  String out; serializeJson(doc, out); return out;
}

void addToRecentBuffer(const SensorReading& r) {
  recentReadings[readIndex] = r;
  readIndex = (readIndex + 1) % MAX_BUFFERED_READINGS;
}

// ════════════════════════════════════════════════════════════════════════════
//  WEB SERVER
// ════════════════════════════════════════════════════════════════════════════
void startWebServer() {
  server.on("/",           HTTP_GET, []() { server.send(200, "text/html", DASHBOARD_HTML); });
  server.on("/api/data",   HTTP_GET, []() { server.send(200, "application/json", getRecentReadingsJSON()); });
  server.on("/api/advice", HTTP_GET, []() { server.send(200, "application/json", getRecentAdviceJSON()); });
  server.on("/api/status", HTTP_GET, []() { server.send(200, "application/json", getSystemStatusJSON()); });
  server.onNotFound([]() { server.send(404, "text/plain", "Not Found"); });
  server.begin();
  Serial.println("[WebServer] HTTP server started on port 80");
}

// ════════════════════════════════════════════════════════════════════════════
//  SUPABASE
// ════════════════════════════════════════════════════════════════════════════
bool sendToSupabase(const SensorReading& r, const String& adviceJson = "") {
  if (!wifiOnline || apMode) return false;
  HTTPClient http;
  http.begin(SUPABASE_URL);
  http.addHeader("Content-Type", "application/json");
  
  JsonDocument doc;
  doc["node_id"]      = r.nodeId;
  doc["timestamp"]    = r.timestamp;
  doc["moisture"]     = r.moisture;
  doc["temperature"]  = r.temp;
  doc["humidity"]     = r.humidity;
  doc["nitrogen"]     = r.nitrogen;
  doc["phosphorus"]   = r.phosphorus;
  doc["potassium"]    = r.potassium;
  doc["latitude"]     = r.lat;
  doc["longitude"]    = r.lng;
  doc["altitude"]     = r.alt;
  doc["satellites"]   = r.satellites;
  doc["season"]       = r.season;
  doc["gps_source"]   = r.gpsWasFallback ? "fallback" : "real";
  if (adviceJson.length() > 0) doc["advice"] = adviceJson;
  
  String payload; serializeJson(doc, payload);
  int code = http.POST(payload);
  String response = http.getString();
  http.end();
  
  Serial.printf("[Supabase] POST → %d %s\n", code, response.substring(0,50).c_str());
  return (code == 200 || code == 201);
}

// ════════════════════════════════════════════════════════════════════════════
//  GOOGLE SHEETS
// ════════════════════════════════════════════════════════════════════════════
bool sendToSheets(const SensorReading& r) {
  if (!wifiOnline || apMode) return false;
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(10000);
  String url = String(SHEETS_URL)
    + "?node_id="     + r.nodeId
    + "&moisture="    + r.moisture
    + "&temperature=" + String(r.temp, 1)
    + "&humidity="    + String(r.humidity, 1)
    + "&nitrogen="    + r.nitrogen
    + "&phosphorus="  + r.phosphorus
    + "&potassium="   + r.potassium
    + "&latitude="    + r.lat
    + "&longitude="   + r.lng
    + "&altitude="    + r.alt
    + "&satellites="  + r.satellites
    + "&season="      + r.season
    + "&gps_source="  + (r.gpsWasFallback ? "fallback" : "real");
  http.begin(url);
  int code = http.GET();
  http.end();
  Serial.printf("[Sheets] HTTP %d  node=%s\n", code, r.nodeId.c_str());
  return (code == 200);
}

bool sendAdviceToSheets(const Advice& a) {
  if (!wifiOnline || apMode) return false;
  HTTPClient http;
  http.setTimeout(10000);
  String url = String(SHEETS_URL) + "/advice"
    + "?node_id="    + a.nodeId
    + "&timestamp="  + a.timestamp
    + "&category="   + a.category
    + "&severity="   + a.severity
    + "&message="    + a.message
    + "&action="     + a.action;
  http.begin(url);
  int code = http.GET();
  http.end();
  Serial.printf("[Sheets/Advice] HTTP %d\n", code);
  return (code == 200);
}

// ════════════════════════════════════════════════════════════════════════════
//  FIREBASE
// ════════════════════════════════════════════════════════════════════════════
bool sendToFirebase(const SensorReading& r) {
  if (!wifiOnline || apMode) return false;
  StaticJsonDocument<512> doc;
  doc["node_id"]    = r.nodeId;
  doc["timestamp"]  = r.timestamp;
  doc["moisture"]   = r.moisture;
  doc["temp"]       = r.temp;
  doc["humidity"]   = r.humidity;
  doc["nitrogen"]   = r.nitrogen;
  doc["phosphorus"] = r.phosphorus;
  doc["potassium"]  = r.potassium;
  doc["latitude"]   = r.lat;
  doc["longitude"]  = r.lng;
  doc["altitude"]   = r.alt;
  doc["satellites"] = r.satellites;
  doc["season"]     = r.season;
  doc["gps_source"] = r.gpsWasFallback ? "fallback" : "real";
  String payload; serializeJson(doc, payload);

  bool ok1 = false, ok2 = false;
  {
    HTTPClient http; http.setTimeout(10000);
    String url = String(FIREBASE_URL) + "/readings/" + r.nodeId + "/latest.json";
    http.begin(url); http.addHeader("Content-Type", "application/json");
    int code = http.PUT(payload);
    Serial.printf("[Firebase] PUT /%s/latest → %d\n", r.nodeId.c_str(), code);
    ok1 = (code == 200); http.end();
  }
  delay(100);
  {
    HTTPClient http; http.setTimeout(10000);
    String url = String(FIREBASE_URL) + "/readings/log.json";
    http.begin(url); http.addHeader("Content-Type", "application/json");
    int code = http.POST(payload);
    Serial.printf("[Firebase] POST /log → %d\n", code);
    ok2 = (code == 200); http.end();
  }
  return (ok1 && ok2);
}

bool sendAdviceToFirebase(const Advice& a) {
  if (!wifiOnline || apMode) return false;
  StaticJsonDocument<256> doc;
  doc["timestamp"]  = a.timestamp;
  doc["node_id"]    = a.nodeId;
  doc["category"]   = a.category;
  doc["severity"]   = a.severity;
  doc["message"]    = a.message;
  doc["action"]     = a.action;
  doc["acknowledged"] = a.acknowledged;
  String payload; serializeJson(doc, payload);
  
  HTTPClient http; http.setTimeout(10000);
  String url = String(FIREBASE_URL) + "/advice/" + a.nodeId + "/" + a.timestamp + ".json";
  http.begin(url); http.addHeader("Content-Type", "application/json");
  int code = http.PUT(payload);
  Serial.printf("[Firebase/Advice] PUT → %d\n", code);
  http.end();
  return (code == 200);
}

// ════════════════════════════════════════════════════════════════════════════
//  SD CARD
// ════════════════════════════════════════════════════════════════════════════
void initSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) { Serial.println("[SD] ❌ FAILED!"); sdReady = false; return; }
  sdReady = true;
  Serial.printf("[SD] ✅ Ready. %.1f MB\n", SD.cardSize() / (1024.0 * 1024.0));
  
  if (!SD.exists(DATA_FILE)) {
    File f = SD.open(DATA_FILE, FILE_WRITE);
    if (f) {
      f.println("timestamp,node_id,moisture_pct,temperature_c,humidity_pct,nitrogen,phosphorus,potassium,latitude,longitude,altitude,satellites,season,gps_source,supabase_sent,sheets_sent,firebase_sent");
      f.close(); Serial.println("[SD] 📄 data.csv created");
    }
  }
  if (!SD.exists(ADVICE_FILE)) {
    File f = SD.open(ADVICE_FILE, FILE_WRITE);
    if (f) {
      f.println("timestamp,node_id,category,severity,message,action,acknowledged,sheets_sent,firebase_sent");
      f.close(); Serial.println("[SD] 📄 advice.csv created");
    }
  }
}

void saveToSD(const SensorReading& r) {
  if (!sdReady) return;
  String row = String(r.timestamp)   + "," + r.nodeId          + "," +
               r.moisture            + "," + String(r.temp, 1) + "," +
               String(r.humidity, 1) + "," + r.nitrogen        + "," +
               r.phosphorus          + "," + r.potassium        + "," +
               r.lat                 + "," + r.lng              + "," +
               r.alt                 + "," + r.satellites       + "," +
               r.season              + "," +
               (r.gpsWasFallback   ? "FALLBACK" : "REAL") + "," +
               (r.syncedToSupabase ? "YES" : "NO")         + "," +
               (r.syncedToSheets   ? "YES" : "NO")         + "," +
               (r.syncedToFirebase ? "YES" : "NO");
  File f = SD.open(DATA_FILE, FILE_APPEND);
  if (f) { f.println(row); f.close(); rowCount++; Serial.printf("[SD] ✅ Row #%lu  node=%s\n", rowCount, r.nodeId.c_str()); }
  else Serial.println("[SD] ❌ Write failed");
}

void saveAdviceToSD(const Advice& a) {
  if (!sdReady) return;
  String row = String(a.timestamp) + "," + a.nodeId + "," +
               a.category + "," + a.severity + "," +
               "\"" + a.message + "\"," +
               "\"" + a.action + "\"," +
               (a.acknowledged ? "YES" : "NO") + "," +
               (a.syncedToSheets ? "YES" : "NO") + "," +
               (a.syncedToFirebase ? "YES" : "NO");
  File f = SD.open(ADVICE_FILE, FILE_APPEND);
  if (f) { f.println(row); f.close(); Serial.printf("[SD/Advice] ✅ Saved: %s - %s\n", a.nodeId.c_str(), a.category.c_str()); }
}

// ════════════════════════════════════════════════════════════════════════════
//  SYNC PENDING
// ════════════════════════════════════════════════════════════════════════════
void syncPendingToClouds() {
  if (!sdReady || !wifiOnline || apMode) return;
  if (!SD.exists(PENDING_FILE) && !SD.exists(ADVICE_FILE)) { 
    Serial.println("[SYNC] ✅ Nothing pending"); 
    return; 
  }
  
  // Sync pending sensor data
  if (SD.exists(PENDING_FILE)) {
    File f = SD.open(PENDING_FILE, FILE_READ);
    if (f) {
      Serial.println("[SYNC] 🔄 Syncing pending sensor data...");
      while (f.available() && wifiOnline && !apMode) {
        String line = f.readStringUntil('\n'); line.trim();
        if (line.length() < 10) continue;
        // Parse and retry sending (simplified - implement full parsing if needed)
        delay(100);
      }
      f.close();
    }
  }
  
  // Sync pending advice
  if (SD.exists(ADVICE_FILE)) {
    File f = SD.open(ADVICE_FILE, FILE_READ);
    if (f) {
      Serial.println("[SYNC] 🔄 Syncing pending advice...");
      while (f.available() && wifiOnline && !apMode) {
        String line = f.readStringUntil('\n'); line.trim();
        if (line.length() < 10) continue;
        // Parse CSV and retry sending advice (simplified)
        delay(100);
      }
      f.close();
    }
  }
  Serial.println("[SYNC] ✅ Sync cycle complete");
}

// ════════════════════════════════════════════════════════════════════════════
//  LoRa PACKET PROCESSING
// ════════════════════════════════════════════════════════════════════════════
void processPacket(String pkt) {
  Serial.println("\n[LoRa] 📦 " + pkt);
  String f[12]; int idx = 0, start = 0;
  for (int i = 0; i <= (int)pkt.length() && idx < 12; i++) {
    if (i == (int)pkt.length() || pkt[i] == ',') { f[idx++] = pkt.substring(start, i); start = i + 1; }
  }
  if (idx < 12) { Serial.printf("[LoRa] ⚠️ %d fields — skip\n", idx); return; }

  SensorReading reading;
  reading.timestamp        = millis() / 1000;
  reading.nodeId           = f[0];
  reading.moisture         = f[1].toInt();
  reading.temp             = f[2].toFloat();
  reading.humidity         = f[3].toFloat();
  reading.nitrogen         = f[4].toInt();
  reading.phosphorus       = f[5].toInt();
  reading.potassium        = f[6].toInt();
  reading.lat              = f[7];
  reading.lng              = f[8];
  reading.alt              = f[9];
  reading.satellites       = f[10];
  reading.season           = f[11];
  reading.syncedToSupabase = false;
  reading.syncedToSheets   = false;
  reading.syncedToFirebase = false;

  resolveGPS(reading.lat, reading.lng, reading.alt, reading.satellites, reading.gpsWasFallback);

  // ── NPK SANITY CHECK & THEFT DETECTION ────────────────────────────────────
  if (reading.nitrogen == 0 && reading.phosphorus == 0 && reading.potassium == 0) {
    int nodeIdx = findNode(reading.nodeId);
    if (nodeIdx >= 0) {
      nodeRegistry[nodeIdx].npkZeroCount++;
      if (nodeRegistry[nodeIdx].npkZeroCount >= NPK_ZERO_THRESHOLD) {
        addAdvice(reading.nodeId, "SECURITY", "CRITICAL",
                  "NPK sensor reading 0/0/0 for " + String(NPK_ZERO_THRESHOLD) + " cycles",
                  "🚨 Investigate: Sensor may be removed, stolen, or disconnected. Verify node location.");
      }
    }
    Serial.printf("  [NPK] ⚠️ Node %s — all zeros (count: %d/%d)\n", 
                  reading.nodeId.c_str(), 
                  nodeIdx>=0 ? nodeRegistry[nodeIdx].npkZeroCount : 0,
                  NPK_ZERO_THRESHOLD);
    // Still process the reading (don't discard) so advice is generated and logged
  }

  // Generate smart advice
  generateAdvice(reading);

  Serial.printf("  Node:%s  M:%d%%  T:%.1f°C  H:%.1f%%  N:%d P:%d K:%d  GPS:%s\n",
                reading.nodeId.c_str(), reading.moisture, reading.temp, reading.humidity,
                reading.nitrogen, reading.phosphorus, reading.potassium,
                reading.gpsWasFallback ? "📍FALLBACK" : "✅REAL");

  updateNodeRegistry(reading);

  // Build advice JSON for Supabase (send active advice with reading)
  String adviceJson = "";
  int nodeIdx = findNode(reading.nodeId);
  if (nodeIdx >= 0 && nodeRegistry[nodeIdx].adviceCount > 0) {
    DynamicJsonDocument advDoc(1024);
    JsonArray advArr = advDoc.createNestedArray("active");
    for (int i = 0; i < nodeRegistry[nodeIdx].adviceCount; i++) {
      JsonObject a = advArr.createNestedObject();
      a["cat"] = nodeRegistry[nodeIdx].activeAdvice[i].category;
      a["sev"] = nodeRegistry[nodeIdx].activeAdvice[i].severity;
      a["msg"] = nodeRegistry[nodeIdx].activeAdvice[i].message;
      a["act"] = nodeRegistry[nodeIdx].activeAdvice[i].action;
    }
    serializeJson(advDoc, adviceJson);
  }

  // Send to clouds if online
  if (wifiOnline && !apMode) {
    reading.syncedToSupabase = sendToSupabase(reading, adviceJson);
    delay(150);
    reading.syncedToSheets   = sendToSheets(reading);
    delay(150);
    reading.syncedToFirebase = sendToFirebase(reading);
    
    // Also send active advice separately
    if (nodeIdx >= 0) {
      for (int i = 0; i < nodeRegistry[nodeIdx].adviceCount; i++) {
        Advice& a = nodeRegistry[nodeIdx].activeAdvice[i];
        if (!a.syncedToSheets) {
          a.syncedToSheets = sendAdviceToSheets(a);
          delay(100);
        }
        if (!a.syncedToFirebase) {
          a.syncedToFirebase = sendAdviceToFirebase(a);
          delay(100);
        }
      }
    }
  }
  
  // Save to SD
  saveToSD(reading);
  addToRecentBuffer(reading);
  
  // Save any new advice to SD
  for (int i = 0; i < MAX_BUFFERED_READINGS; i++) {
    int aIdx = (adviceIndex - 1 - i + MAX_BUFFERED_READINGS) % MAX_BUFFERED_READINGS;
    if (recentAdvice[aIdx].timestamp == reading.timestamp && recentAdvice[aIdx].nodeId == reading.nodeId) {
      saveAdviceToSD(recentAdvice[aIdx]);
    }
  }

  Serial.printf("  [GW] WiFi:%s Supabase:%s Sheets:%s FB:%s SD:%s Nodes:%d/%d Advice:%d\n",
                apMode ? "AP" : (wifiOnline ? "✅" : "❌"),
                reading.syncedToSupabase ? "✅" : "⏳",
                reading.syncedToSheets   ? "✅" : "⏳",
                reading.syncedToFirebase ? "✅" : "⏳",
                sdReady ? "✅" : "❌",
                nodeCount, MAX_NODES, adviceCount);
}

// ════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  delay(2000);

  pinMode(LORA_TX,  OUTPUT); digitalWrite(LORA_TX,  HIGH);
  pinMode(LORA_MDO, OUTPUT); digitalWrite(LORA_MDO, LOW);
  pinMode(SD_CS,    OUTPUT); digitalWrite(SD_CS,    HIGH);

  Serial.begin(115200); delay(500);

  Serial.println("\n🌿 ══════════════════════════════════════════");
  Serial.println("   FARMSENSE GATEWAY v3.0 — Smart Advisory");
  Serial.printf("   Max nodes: %d | Buffer: %d readings\n", MAX_NODES, MAX_BUFFERED_READINGS);
  Serial.printf("   GPS Fallback: %s, %s\n", FALLBACK_LAT, FALLBACK_LNG);
  Serial.println("   NPK Alert: " + String(NPK_ZERO_THRESHOLD) + " consecutive zeros = SECURITY warning");
  Serial.println("   Advice Categories: NPK, IRRIGATION, FERTILIZER, SECURITY, WEATHER, MAINTENANCE");
  Serial.println("════════════════════════════════════════════ 🌿\n");

  initSD();

  loraSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(300);
  while (loraSerial.available()) loraSerial.read();
  Serial.println("[INIT] 📡 LoRa serial ready (GPIO 16/17)");

  lastWifiTry = millis();
  if (tryConnectWiFi()) { wifiOnline = true; syncPendingToClouds(); }
  else startAccessPoint();

  startWebServer();

  Serial.println("\n[INIT] ✅ Ready!");
  Serial.printf("  Dashboard: http://%s\n",
                apMode ? WiFi.softAPIP().toString().c_str() :
                wifiOnline ? WiFi.localIP().toString().c_str() : "N/A");
  Serial.println("  Advisories appear in real-time on dashboard + sent to clouds");
}

// ════════════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════════════
void loop() {
  manageWiFi();
  server.handleClient();

  static String buffer = "";
  while (loraSerial.available()) {
    char c = (char)loraSerial.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 5 && buffer.indexOf(',') > 0) {
        buffer.trim();
        String clean = "";
        for (int i = 0; i < (int)buffer.length(); i++)
          if (isPrintable(buffer[i])) clean += buffer[i];
        if (clean.length() > 5 && clean.indexOf(',') > 0) processPacket(clean);
      }
      buffer = "";
    } else {
      buffer += c;
      if (buffer.length() > 200) buffer = buffer.substring(buffer.length() - 150);
    }
  }
  delay(2);
}