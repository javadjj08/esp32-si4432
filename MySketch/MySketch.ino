/*
 * Si4432 RF Scanner / Recorder / Replayer
 * ESP32 DevKit V1 + Si4432 (BPS1EZ - Rev B1)
 * UI: WebSocket + Mobile Browser
 * OTA: ElegantOTA (from browser)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <LittleFS.h>
#include <RadioLib.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>

// ==================== پین‌های Si4432 ====================
#define PIN_CS   5
#define PIN_IRQ  16
#define PIN_SDN  15
#define PIN_GPIO 4

Si4432 radio = new Module(PIN_CS, PIN_IRQ, PIN_SDN, PIN_GPIO);

// ==================== WiFi ====================
const char* STA_SSID = "YOUR_WIFI_SSID";
const char* STA_PASS = "YOUR_WIFI_PASSWORD";
const char* AP_SSID  = "RF-Scanner";
const char* AP_PASS  = "12345678";

// ==================== تنظیمات اسکن ====================
float scanStart   = 430.0;
float scanEnd     = 440.0;
float scanStep    = 0.1;
float currentFreq = 430.0;
bool  scanning    = false;
int   rssiThreshold = -75;

// ==================== ضبط ====================
#define MAX_RECORD 512
uint8_t  recordBuf[MAX_RECORD];
uint16_t recordLen  = 0;
bool     recording  = false;
float    recordFreq = 0.0;
unsigned long lastRecvTime = 0;
#define RECORD_TIMEOUT_MS 400

// ==================== ایندکس سیگنال‌ها ====================
#define MAX_SIGNALS 16
#define INDEX_FILE  "/idx.bin"

struct SigEntry {
  uint16_t id;
  float    freq;
  uint16_t len;
};

SigEntry sigIndex[MAX_SIGNALS];
uint16_t sigCount = 0;
uint16_t nextSigId = 1;

// ==================== وب ====================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ==================== HTML UI ====================
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="fa" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RF Scanner</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Tahoma,sans-serif;background:#0a0e17;color:#e0e6ed;padding:16px;max-width:480px;margin:auto}
h1{font-size:1.2em;text-align:center;color:#00e5ff;margin-bottom:14px}
.card{background:#151c2c;border-radius:12px;padding:14px;margin-bottom:12px;border:1px solid #1e2a45}
.big{font-size:2.4em;font-weight:700;color:#00e5ff;font-family:monospace;text-align:center;display:block}
.unit{text-align:center;color:#7a8ba8;font-size:.9em}
.rssi-bar{height:8px;background:#1e2a45;border-radius:4px;margin-top:10px;overflow:hidden}
.rssi-fill{height:100%;width:0;background:#00e5ff;transition:width .15s}
.status{text-align:center;font-size:.85em;color:#7a8ba8;margin-top:8px}
.btn-row{display:flex;gap:8px;margin-top:6px}
button{flex:1;padding:14px;border:none;border-radius:10px;font-size:1em;font-weight:600;cursor:pointer}
.btn-scan{background:#00e5ff;color:#0a0e17}
.btn-record{background:#ff3d71;color:#fff}
.btn-stop{background:#2d3a54;color:#e0e6ed}
.btn-replay{background:#00b894;color:#fff;padding:8px 16px;font-size:.85em;flex:0 0 auto}
.signal-item{display:flex;justify-content:space-between;align-items:center;padding:10px 4px;border-bottom:1px solid #1e2a45;font-family:monospace;font-size:.9em}
.signal-item:last-child{border-bottom:none}
.signal-freq{color:#00e5ff}
.signal-meta{color:#7a8ba8;font-size:.8em}
.empty{color:#3a4a66;text-align:center;padding:16px}
</style>
</head>
<body>
<h1>📡 اسکنر فرکانس Si4432</h1>

<div class="card">
  <span class="big" id="freq">---.--</span>
  <div class="unit">MHz</div>
  <div class="rssi-bar"><div class="rssi-fill" id="rssiBar"></div></div>
  <div class="status" id="rssiTxt">RSSI: --- dBm</div>
  <div class="status" id="statusTxt">آماده</div>
</div>

<div class="card">
  <div class="btn-row">
    <button class="btn-scan" onclick="send('SCAN')">▶ اسکن</button>
    <button class="btn-record" onclick="send('RECORD')">● ضبط</button>
    <button class="btn-stop" onclick="send('STOP')">■ توقف</button>
  </div>
</div>

<div class="card">
  <div style="color:#7a8ba8;font-size:.9em;margin-bottom:6px">سیگنال‌های ضبط شده</div>
  <div id="list"><div class="empty">خالی</div></div>
</div>

<script>
let ws;
function connect(){
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen = () => { setStatus('متصل'); send('LIST'); };
  ws.onclose = () => { setStatus('قطع — تلاش مجدد'); setTimeout(connect, 2000); };
  ws.onmessage = (e) => {
    const m = JSON.parse(e.data);
    if (m.type === 'SCAN') {
      document.getElementById('freq').textContent = m.freq.toFixed(2);
      const pct = Math.max(0, Math.min(100, (m.rssi + 100) * 1.5));
      const bar = document.getElementById('rssiBar');
      bar.style.width = pct + '%';
      bar.style.background = m.rssi > -70 ? '#00e5ff' : m.rssi > -85 ? '#f9a825' : '#ff3d71';
      document.getElementById('rssiTxt').textContent = 'RSSI: ' + m.rssi + ' dBm';
    }
    if (m.type === 'SIGNALS') renderList(m.list);
    if (m.type === 'STATUS') setStatus(m.msg);
  };
}
function send(cmd){ if (ws && ws.readyState === 1) ws.send(cmd); }
function setStatus(s){ document.getElementById('statusTxt').textContent = s; }
function renderList(list){
  const el = document.getElementById('list');
  if (!list || !list.length){ el.innerHTML = '<div class="empty">خالی</div>'; return; }
  el.innerHTML = list.map(s =>
    '<div class="signal-item"><div><div class="signal-freq">' + s.freq.toFixed(2) + ' MHz</div>' +
    '<div class="signal-meta">' + s.len + ' بایت</div></div>' +
    '<button class="btn-replay" onclick="send(\'REPLAY:' + s.id + '\')">پخش</button></div>'
  ).join('');
}
connect();
</script>
</body>
</html>
)HTMLPAGE";

// ==================== اطلاع به UI ====================
void notifyScan(float freq, int rssi) {
  StaticJsonDocument<96> doc;
  doc["type"] = "SCAN";
  doc["freq"] = freq;
  doc["rssi"] = rssi;
  String out; serializeJson(doc, out);
  ws.textAll(out);
}

void notifyStatus(const String& msg) {
  StaticJsonDocument<160> doc;
  doc["type"] = "STATUS";
  doc["msg"]  = msg;
  String out; serializeJson(doc, out);
  ws.textAll(out);
}

void notifySignalList() {
  StaticJsonDocument<768> doc;
  doc["type"] = "SIGNALS";
  JsonArray arr = doc.createNestedArray("list");
  for (uint16_t i = 0; i < sigCount; i++) {
    JsonObject o = arr.createNestedObject();
    o["id"]   = sigIndex[i].id;
    o["freq"] = sigIndex[i].freq;
    o["len"]  = sigIndex[i].len;
  }
  String out; serializeJson(doc, out);
  ws.textAll(out);
}

// ==================== LittleFS ====================
void loadIndex() {
  sigCount = 0;
  File f = LittleFS.open(INDEX_FILE, "r");
  if (!f) return;
  f.read((uint8_t*)&sigCount, sizeof(sigCount));
  if (sigCount > MAX_SIGNALS) { sigCount = 0; f.close(); return; }
  for (uint16_t i = 0; i < sigCount; i++) {
    f.read((uint8_t*)&sigIndex[i], sizeof(SigEntry));
    if (sigIndex[i].id >= nextSigId) nextSigId = sigIndex[i].id + 1;
  }
  f.close();
}

void saveIndex() {
  File f = LittleFS.open(INDEX_FILE, "w");
  if (!f) return;
  f.write((uint8_t*)&sigCount, sizeof(sigCount));
  for (uint16_t i = 0; i < sigCount; i++) {
    f.write((uint8_t*)&sigIndex[i], sizeof(SigEntry));
  }
  f.close();
}

bool saveSignal(float freq, uint8_t* data, uint16_t len) {
  if (sigCount >= MAX_SIGNALS) return false;
  uint16_t id = nextSigId++;
  char path[24];
  snprintf(path, sizeof(path), "/sig_%u.bin", id);
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  f.write(data, len);
  f.close();
  sigIndex[sigCount] = { id, freq, len };
  sigCount++;
  saveIndex();
  return true;
}

bool loadSignal(uint16_t id, float* freq, uint8_t* buf, uint16_t* len) {
  for (uint16_t i = 0; i < sigCount; i++) {
    if (sigIndex[i].id == id) {
      char path[24];
      snprintf(path, sizeof(path), "/sig_%u.bin", id);
      File f = LittleFS.open(path, "r");
      if (!f) return false;
      *len  = sigIndex[i].len;
      *freq = sigIndex[i].freq;
      f.read(buf, *len);
      f.close();
      return true;
    }
  }
  return false;
}

// ==================== Si4432 ====================
void initRadio() {
  Serial.print(F("[Si4432] init... "));
  int state = radio.begin(433.0, 4.8, 5.0, 181.1, 20, 16, 0x2D, 16);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("failed, code %d\n", state);
    return;
  }
  radio.setFrequency(433.0);
  radio.startReceive();
  Serial.println(F("OK"));
}

void doScan() {
  if (!scanning) return;
  if (currentFreq > scanEnd) currentFreq = scanStart;

  radio.setFrequency(currentFreq);
  radio.startReceive();
  delay(6);

  float rssi = radio.getRSSI();
  notifyScan(currentFreq, (int)rssi);

  currentFreq += scanStep;
}

// ==================== ضبط ====================
void startRecording() {
  scanning  = false;
  recording = true;
  recordLen = 0;
  recordFreq = currentFreq;
  lastRecvTime = millis();
  radio.setFrequency(recordFreq);
  radio.startReceive();
  notifyStatus("در حال ضبط روی " + String(recordFreq, 2) + " MHz");
}

void doRecord() {
  if (!recording) return;

  if (radio.available()) {
    uint8_t buf[64];
    size_t len = radio.readData(buf, sizeof(buf));
    if (len > 0 && (recordLen + len) <= MAX_RECORD) {
      memcpy(recordBuf + recordLen, buf, len);
      recordLen += len;
      lastRecvTime = millis();
    }
  }

  if (recordLen > 0 && (millis() - lastRecvTime > RECORD_TIMEOUT_MS)) {
    finishRecording();
  }
}

void finishRecording() {
  recording = false;
  if (recordLen == 0) {
    notifyStatus("داده‌ای دریافت نشد");
    return;
  }
  if (saveSignal(recordFreq, recordBuf, recordLen)) {
    notifyStatus("ذخیره شد: " + String(recordLen) + " بایت");
    notifySignalList();
  } else {
    notifyStatus("حافظه پر است");
  }
  recordLen = 0;
}

// ==================== بازپخش ====================
void replaySignal(float freq, uint8_t* data, uint16_t len) {
  radio.setFrequency(freq);
  delay(10);
  radio.startTransmit();
  delay(10);
  radio.transmit(data, len);
  radio.finishTransmit();
  delay(20);
  radio.startReceive();
  notifyStatus("پخش شد: " + String(freq, 2) + " MHz");
}

// ==================== WebSocket ====================
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type != WS_EVT_DATA) return;
  String cmd;
  cmd.reserve(len + 1);
  for (size_t i = 0; i < len; i++) cmd += (char)data[i];
  cmd.trim();

  if (cmd == "SCAN") {
    scanning = true;
    recording = false;
    currentFreq = scanStart;
    notifyStatus("در حال اسکن...");
  }
  else if (cmd == "RECORD") {
    startRecording();
  }
  else if (cmd == "STOP") {
    scanning = false;
    if (recording) finishRecording();
    notifyStatus("متوقف");
  }
  else if (cmd == "LIST") {
    notifySignalList();
  }
  else if (cmd.startsWith("REPLAY:")) {
    uint16_t id = cmd.substring(7).toInt();
    float freq; uint8_t buf[MAX_RECORD]; uint16_t blen;
    if (loadSignal(id, &freq, buf, &blen)) {
      replaySignal(freq, buf, blen);
    } else {
      notifyStatus("سیگنال یافت نشد");
    }
  }
}

// ==================== WiFi ====================
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print(F("[WiFi] Connecting"));
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500); Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] IP: " + WiFi.localIP().toString());
  } else {
    Serial.println(F("\n[WiFi] STA failed, starting AP"));
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.println("[WiFi] AP IP: " + WiFi.softAPIP().toString());
  }
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== Si4432 RF Controller ==="));

  if (!LittleFS.begin(true)) {
    Serial.println(F("[FS] mount failed"));
  }
  loadIndex();

  initWiFi();
  initRadio();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });

  ElegantOTA.begin(&server);
  server.begin();

  Serial.println("[HTTP] http://" + WiFi.localIP().toString() + "/");
  Serial.println("[OTA]  http://" + WiFi.localIP().toString() + "/update");
}

// ==================== Loop ====================
void loop() {
  ElegantOTA.loop();
  ws.cleanupClients();

  if (scanning)  doScan();
  if (recording) doRecord();

  delay(1);
}
