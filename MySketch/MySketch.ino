/*
 * Si4432 All-in-One RF Scanner with SDR & Dynamic Pin Config
 * ESP32 DevKit V1 + Si4432 (BPS1EZ)
 * UI: WebSocket + Mobile Browser
 * OTA: ElegantOTA
 */

#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

// ==================== پیش‌فرض پین‌ها ====================
int PIN_CS   = 5;
int PIN_IRQ  = 16;
int PIN_SDN  = 15;
int PIN_GPIO = 4;

Si4432* radio = nullptr;

// ==================== WiFi ====================
const char* STA_SSID = "YOUR_WIFI_SSID";
const char* STA_PASS = "YOUR_WIFI_PASSWORD";
const char* AP_SSID  = "RF-Scanner";
const char* AP_PASS  = "12345678";

// ==================== تنظیمات ذخیره‌شده ====================
Preferences prefs;
float cfgFreq = 433.0;
float cfgBitrate = 4.8;
float cfgFreqDev = 5.0;
float cfgRxBw = 181.1;
int8_t cfgPower = 20;
uint8_t cfgPreamble = 16;
int cfgRssiThreshold = -75;
float scanStart = 430.0;
float scanEnd   = 440.0;
float scanStep  = 0.1;
bool  cfgSDRMode = false;

// ==================== وضعیت داخلی ====================
bool scanning = false;
bool recording = false;
float currentFreq = 430.0;
float recordFreq = 0;
#define MAX_RECORD 512
uint8_t  recordBuf[MAX_RECORD];
uint16_t recordLen = 0;
unsigned long lastRecvTime = 0;
#define RECORD_TIMEOUT_MS 400

// ==================== ایندکس سیگنال‌ها ====================
#define MAX_SIGNALS 16
struct SigEntry { uint16_t id; float freq; uint16_t len; };
SigEntry sigIndex[MAX_SIGNALS];
uint16_t sigCount = 0;
uint16_t nextSigId = 1;

// ==================== وب ====================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocket audioWs("/audio"); // WebSocket جداگانه برای صدا

// ==================== HTML UI (کامل) ====================
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="fa" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RF Scanner Pro</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Tahoma,sans-serif;background:#0a0e17;color:#e0e6ed;padding:12px;max-width:520px;margin:auto}
h1{font-size:1.1em;text-align:center;color:#00e5ff;margin-bottom:12px}
.card{background:#151c2c;border-radius:12px;padding:12px;margin-bottom:10px;border:1px solid #1e2a45}
.row{display:flex;gap:8px;margin-bottom:8px;align-items:center}
.label{color:#7a8ba8;font-size:.8em;min-width:70px}
input[type=number],input[type=text],select{flex:1;background:#0a0e17;color:#0f0;border:1px solid #1e2a45;border-radius:6px;padding:8px;font-family:monospace}
button{flex:1;padding:12px;border:none;border-radius:8px;font-size:.95em;font-weight:600;cursor:pointer;margin:2px}
.btn-scan{background:#00e5ff;color:#0a0e17}
.btn-record{background:#ff3d71;color:#fff}
.btn-stop{background:#2d3a54;color:#e0e6ed}
.btn-replay{background:#00b894;color:#fff;padding:8px 14px;font-size:.8em}
.btn-settings{background:#6c5ce7;color:#fff}
.tabs{display:flex;gap:4px;margin-bottom:10px}
.tab{flex:1;padding:10px;text-align:center;background:#1e2a45;border-radius:8px 8px 0 0;cursor:pointer;font-size:.85em}
.tab.active{background:#00e5ff;color:#0a0e17;font-weight:bold}
.tab-content{display:none}
.tab-content.active{display:block}
.freq-display{text-align:center;padding:10px 0}
.big{font-size:2.2em;font-weight:700;color:#00e5ff;font-family:monospace}
.unit{color:#7a8ba8;font-size:.85em}
.rssi-bar{height:6px;background:#1e2a45;border-radius:3px;margin-top:8px;overflow:hidden}
.rssi-fill{height:100%;width:0;background:#00e5ff;transition:width .15s}
.status{text-align:center;font-size:.8em;color:#7a8ba8;margin-top:6px}
.signal-item{display:flex;justify-content:space-between;align-items:center;padding:8px 4px;border-bottom:1px solid #1e2a45;font-family:monospace;font-size:.85em}
.signal-item:last-child{border-bottom:none}
.signal-freq{color:#00e5ff}
.signal-meta{color:#7a8ba8;font-size:.75em}
.empty{color:#3a4a66;text-align:center;padding:16px}
.settings-section{margin-bottom:12px}
.settings-section h3{color:#00e5ff;font-size:.9em;margin-bottom:8px;border-bottom:1px solid #1e2a45;padding-bottom:4px}
#audioStatus{font-size:.8em;color:#7a8ba8;text-align:center;padding:8px}
</style>
</head>
<body>
<h1>📡 Si4432 Scanner Pro</h1>

<div class="tabs">
  <div class="tab active" onclick="switchTab('scan')">اسکن</div>
  <div class="tab" onclick="switchTab('sdr')">SDR</div>
  <div class="tab" onclick="switchTab('settings')">تنظیمات</div>
</div>

<!-- ========== تب اسکن ========== -->
<div id="tab-scan" class="tab-content active">
  <div class="card">
    <div class="freq-display">
      <span class="big" id="freq">---.--</span>
      <span class="unit">MHz</span>
    </div>
    <div class="rssi-bar"><div class="rssi-fill" id="rssiBar"></div></div>
    <div class="status" id="rssiTxt">RSSI: --- dBm</div>
    <div class="status" id="statusTxt">آماده</div>
  </div>

  <div class="card">
    <div class="row">
      <span class="label">فرکانس دستی</span>
      <input type="number" id="manualFreq" step="0.1" value="433.0" min="240" max="930">
      <button class="btn-scan" onclick="setManualFreq()">تنظیم</button>
    </div>
    <div class="row">
      <button class="btn-scan" onclick="send('SCAN')">▶ اسکن</button>
      <button class="btn-record" onclick="send('RECORD')">● ضبط</button>
      <button class="btn-stop" onclick="send('STOP')">■ توقف</button>
    </div>
  </div>

  <div class="card">
    <div style="color:#7a8ba8;font-size:.85em;margin-bottom:6px">سیگنال‌های ضبط شده</div>
    <div id="list"><div class="empty">خالی</div></div>
  </div>
</div>

<!-- ========== تب SDR ========== -->
<div id="tab-sdr" class="tab-content">
  <div class="card">
    <div class="freq-display">
      <span class="big" id="sdrFreq">---.--</span>
      <span class="unit">MHz</span>
    </div>
    <div id="audioStatus">برای شروع پخش، دکمه را بزنید</div>
    <button class="btn-scan" onclick="toggleSDR()" id="sdrBtn">▶ شروع SDR</button>
  </div>
  <div class="card">
    <div class="row">
      <span class="label">فرکانس</span>
      <input type="number" id="sdrFreqInput" step="0.1" value="433.0" min="240" max="930">
      <button class="btn-settings" onclick="setSDRFreq()">تنظیم</button>
    </div>
    <div class="row">
      <span class="label">مدولاسیون</span>
      <select id="sdrMod">
        <option value="FSK">FSK</option>
        <option value="GFSK">GFSK</option>
        <option value="OOK">OOK</option>
      </select>
    </div>
  </div>
</div>

<!-- ========== تب تنظیمات ========== -->
<div id="tab-settings" class="tab-content">
  <div class="card settings-section">
    <h3>📻 پارامترهای رادیو</h3>
    <div class="row"><span class="label">فرکانس پایه</span><input type="number" id="setFreq" step="0.1" value="433.0" min="240" max="930"></div>
    <div class="row"><span class="label">نرخ بیت (kbps)</span><input type="number" id="setBitrate" step="0.1" value="4.8"></div>
    <div class="row"><span class="label">انحراف فرکانس</span><input type="number" id="setFreqDev" step="0.1" value="5.0"></div>
    <div class="row"><span class="label">پهنای باند RX</span><input type="number" id="setRxBw" step="0.1" value="181.1"></div>
    <div class="row"><span class="label">توان (dBm)</span><input type="number" id="setPower" step="3" value="20" min="-1" max="20"></div>
    <div class="row"><span class="label">مدولاسیون</span><select id="setMod"><option value="FSK">FSK</option><option value="GFSK">GFSK</option><option value="OOK">OOK</option></select></div>
  </div>

  <div class="card settings-section">
    <h3>🔍 تنظیمات اسکن</h3>
    <div class="row"><span class="label">شروع (MHz)</span><input type="number" id="setScanStart" step="0.1" value="430.0"></div>
    <div class="row"><span class="label">پایان (MHz)</span><input type="number" id="setScanEnd" step="0.1" value="440.0"></div>
    <div class="row"><span class="label">گام (MHz)</span><input type="number" id="setScanStep" step="0.05" value="0.1"></div>
    <div class="row"><span class="label">آستانه RSSI</span><input type="number" id="setRssiTh" value="-75" min="-120" max="0"></div>
  </div>

  <div class="card settings-section">
    <h3>📌 پین‌های Si4432</h3>
    <div class="row"><span class="label">CS</span><input type="number" id="setPinCS" value="5"></div>
    <div class="row"><span class="label">IRQ</span><input type="number" id="setPinIRQ" value="16"></div>
    <div class="row"><span class="label">SDN</span><input type="number" id="setPinSDN" value="15"></div>
    <div class="row"><span class="label">GPIO</span><input type="number" id="setPinGPIO" value="4"></div>
  </div>

  <div class="card settings-section">
    <h3>📶 WiFi</h3>
    <div class="row"><span class="label">SSID</span><input type="text" id="setSsid" value="YOUR_WIFI_SSID"></div>
    <div class="row"><span class="label">رمز</span><input type="text" id="setPass" value="YOUR_WIFI_PASSWORD"></div>
  </div>

  <div class="card">
    <button class="btn-settings" onclick="saveSettings()">💾 ذخیره تنظیمات</button>
    <button class="btn-stop" onclick="resetSettings()">↺ بازگشت به پیش‌فرض</button>
  </div>
</div>

<script>
let ws, audioWs, sdrActive = false, audioCtx = null, audioSource = null;

// ===== اتصال WebSocket =====
function connectWS() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen = () => { setStatus('متصل'); send('LIST'); loadSettings(); };
  ws.onclose = () => { setStatus('قطع — تلاش مجدد'); setTimeout(connectWS, 2000); };
  ws.onmessage = (e) => {
    try {
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
      if (m.type === 'SETTINGS') applySettings(m.data);
      if (m.type === 'SDR_FREQ') document.getElementById('sdrFreq').textContent = m.freq.toFixed(2);
    } catch(err) { console.log('bad json', e.data); }
  };
}

// ===== ارسال فرمان =====
function send(cmd) { if (ws && ws.readyState === 1) ws.send(cmd); }

// ===== تنظیم فرکانس دستی =====
function setManualFreq() {
  const f = parseFloat(document.getElementById('manualFreq').value);
  if (f >= 240 && f <= 930) send('SET_FREQ:' + f);
}

// ===== تغییر تب =====
function switchTab(name) {
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.querySelectorAll('.tab-content').forEach(t => t.classList.remove('active'));
  document.querySelector('.tab[onclick*="' + name + '"]').classList.add('active');
  document.getElementById('tab-' + name).classList.add('active');
}

// ===== SDR =====
function toggleSDR() {
  if (!sdrActive) {
    // اتصال WebSocket صوتی
    audioWs = new WebSocket('ws://' + location.host + '/audio');
    audioWs.binaryType = 'arraybuffer';
    audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    audioSource = audioCtx.createBufferSource();

    audioWs.onmessage = (e) => {
      if (audioCtx.state === 'suspended') audioCtx.resume();
      const data = new Float32Array(e.data);
      const buf = audioCtx.createBuffer(1, data.length, audioCtx.sampleRate);
      buf.copyToChannel(data, 0);
      const src = audioCtx.createBufferSource();
      src.buffer = buf;
      src.connect(audioCtx.destination);
      src.start();
    };

    audioWs.onopen = () => {
      send('SDR_START');
      sdrActive = true;
      document.getElementById('sdrBtn').textContent = '⏹ توقف SDR';
      document.getElementById('audioStatus').textContent = 'SDR فعال — صدا در حال پخش';
    };

    audioWs.onclose = () => {
      sdrActive = false;
      document.getElementById('sdrBtn').textContent = '▶ شروع SDR';
      document.getElementById('audioStatus').textContent = 'قطع شد';
    };
  } else {
    if (audioWs) audioWs.close();
    send('SDR_STOP');
    sdrActive = false;
    document.getElementById('sdrBtn').textContent = '▶ شروع SDR';
    document.getElementById('audioStatus').textContent = 'متوقف شد';
  }
}

function setSDRFreq() {
  const f = parseFloat(document.getElementById('sdrFreqInput').value);
  if (f >= 240 && f <= 930) send('SDR_FREQ:' + f);
}

// ===== لیست سیگنال‌ها =====
function renderList(list) {
  const el = document.getElementById('list');
  if (!list || !list.length) { el.innerHTML = '<div class="empty">خالی</div>'; return; }
  el.innerHTML = list.map(s =>
    '<div class="signal-item"><div><div class="signal-freq">' + s.freq.toFixed(2) + ' MHz</div>' +
    '<div class="signal-meta">' + s.len + ' bytes</div></div>' +
    '<button class="btn-replay" onclick="send(\'REPLAY:' + s.id + '\')">پخش</button></div>'
  ).join('');
}

// ===== تنظیمات =====
function loadSettings() { send('GET_SETTINGS'); }

function applySettings(d) {
  document.getElementById('setFreq').value = d.freq;
  document.getElementById('setBitrate').value = d.bitrate;
  document.getElementById('setFreqDev').value = d.freqDev;
  document.getElementById('setRxBw').value = d.rxBw;
  document.getElementById('setPower').value = d.power;
  document.getElementById('setMod').value = d.modulation;
  document.getElementById('setScanStart').value = d.scanStart;
  document.getElementById('setScanEnd').value = d.scanEnd;
  document.getElementById('setScanStep').value = d.scanStep;
  document.getElementById('setRssiTh').value = d.rssiThreshold;
  document.getElementById('setPinCS').value = d.pinCS;
  document.getElementById('setPinIRQ').value = d.pinIRQ;
  document.getElementById('setPinSDN').value = d.pinSDN;
  document.getElementById('setPinGPIO').value = d.pinGPIO;
  document.getElementById('setSsid').value = d.ssid;
  document.getElementById('setPass').value = d.pass;
}

function saveSettings() {
  const s = {
    freq: parseFloat(document.getElementById('setFreq').value),
    bitrate: parseFloat(document.getElementById('setBitrate').value),
    freqDev: parseFloat(document.getElementById('setFreqDev').value),
    rxBw: parseFloat(document.getElementById('setRxBw').value),
    power: parseInt(document.getElementById('setPower').value),
    modulation: document.getElementById('setMod').value,
    scanStart: parseFloat(document.getElementById('setScanStart').value),
    scanEnd: parseFloat(document.getElementById('setScanEnd').value),
    scanStep: parseFloat(document.getElementById('setScanStep').value),
    rssiThreshold: parseInt(document.getElementById('setRssiTh').value),
    pinCS: parseInt(document.getElementById('setPinCS').value),
    pinIRQ: parseInt(document.getElementById('setPinIRQ').value),
    pinSDN: parseInt(document.getElementById('setPinSDN').value),
    pinGPIO: parseInt(document.getElementById('setPinGPIO').value),
    ssid: document.getElementById('setSsid').value,
    pass: document.getElementById('setPass').value
  };
  send('SAVE_SETTINGS:' + JSON.stringify(s));
}

function resetSettings() {
  if (confirm('همه تنظیمات به حالت پیش‌فرض برگردد؟')) send('RESET_SETTINGS');
}

function setStatus(s) { document.getElementById('statusTxt').textContent = s; }

connectWS();
</script>
</body>
</html>
)HTMLPAGE";

// ==================== forward ====================
void finishRecording();
void reinitRadio();
void loadSettings();
void saveSettingsToPrefs();

// ==================== JSON Helpers ====================
void notifyScan(float freq, int rssi) {
  String out = "{\"type\":\"SCAN\",\"freq\":" + String(freq, 2) + ",\"rssi\":" + String(rssi) + "}";
  ws.textAll(out);
}
void notifyStatus(const String& msg) {
  String s = msg; s.replace("\\", "\\\\"); s.replace("\"", "\\\"");
  ws.textAll("{\"type\":\"STATUS\",\"msg\":\"" + s + "\"}");
}
void notifySignalList() {
  String out = "{\"type\":\"SIGNALS\",\"list\":[";
  for (uint16_t i = 0; i < sigCount; i++) {
    if (i) out += ",";
    out += "{\"id\":" + String(sigIndex[i].id) + ",\"freq\":" + String(sigIndex[i].freq, 2) + ",\"len\":" + String(sigIndex[i].len) + "}";
  }
  out += "]}";
  ws.textAll(out);
}
void notifySettings() {
  String out = "{\"type\":\"SETTINGS\",\"data\":{";
  out += "\"freq\":" + String(cfgFreq, 2);
  out += ",\"bitrate\":" + String(cfgBitrate, 2);
  out += ",\"freqDev\":" + String(cfgFreqDev, 2);
  out += ",\"rxBw\":" + String(cfgRxBw, 2);
  out += ",\"power\":" + String(cfgPower);
  out += ",\"modulation\":\"FSK\"";
  out += ",\"scanStart\":" + String(scanStart, 2);
  out += ",\"scanEnd\":" + String(scanEnd, 2);
  out += ",\"scanStep\":" + String(scanStep, 2);
  out += ",\"rssiThreshold\":" + String(cfgRssiThreshold);
  out += ",\"pinCS\":" + String(PIN_CS) + ",\"pinIRQ\":" + String(PIN_IRQ) + ",\"pinSDN\":" + String(PIN_SDN) + ",\"pinGPIO\":" + String(PIN_GPIO);
  out += ",\"ssid\":\"" + String(STA_SSID) + "\",\"pass\":\"" + String(STA_PASS) + "\"";
  out += "}}";
  ws.textAll(out);
}

// ==================== Preferences ====================
void loadSettings() {
  prefs.begin("rfcfg", true);
  cfgFreq          = prefs.getFloat("freq", 433.0);
  cfgBitrate       = prefs.getFloat("bitrate", 4.8);
  cfgFreqDev       = prefs.getFloat("freqDev", 5.0);
  cfgRxBw          = prefs.getFloat("rxBw", 181.1);
  cfgPower         = prefs.getInt("power", 20);
  cfgPreamble      = prefs.getUInt("preamble", 16);
  cfgRssiThreshold = prefs.getInt("rssiTh", -75);
  scanStart        = prefs.getFloat("scanStart", 430.0);
  scanEnd          = prefs.getFloat("scanEnd", 440.0);
  scanStep         = prefs.getFloat("scanStep", 0.1);
  PIN_CS           = prefs.getInt("pinCS", 5);
  PIN_IRQ          = prefs.getInt("pinIRQ", 16);
  PIN_SDN          = prefs.getInt("pinSDN", 15);
  PIN_GPIO         = prefs.getInt("pinGPIO", 4);
  prefs.end();
}

void saveSettingsToPrefs() {
  prefs.begin("rfcfg", false);
  prefs.putFloat("freq", cfgFreq);
  prefs.putFloat("bitrate", cfgBitrate);
  prefs.putFloat("freqDev", cfgFreqDev);
  prefs.putFloat("rxBw", cfgRxBw);
  prefs.putInt("power", cfgPower);
  prefs.putUInt("preamble", cfgPreamble);
  prefs.putInt("rssiTh", cfgRssiThreshold);
  prefs.putFloat("scanStart", scanStart);
  prefs.putFloat("scanEnd", scanEnd);
  prefs.putFloat("scanStep", scanStep);
  prefs.putInt("pinCS", PIN_CS);
  prefs.putInt("pinIRQ", PIN_IRQ);
  prefs.putInt("pinSDN", PIN_SDN);
  prefs.putInt("pinGPIO", PIN_GPIO);
  prefs.end();
}

// ==================== LittleFS ====================
void loadIndex() {
  sigCount = 0;
  File f = LittleFS.open("/idx.bin", "r");
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
  File f = LittleFS.open("/idx.bin", "w");
  if (!f) return;
  f.write((uint8_t*)&sigCount, sizeof(sigCount));
  for (uint16_t i = 0; i < sigCount; i++) f.write((uint8_t*)&sigIndex[i], sizeof(SigEntry));
  f.close();
}
bool saveSignal(float freq, uint8_t* data, uint16_t len) {
  if (sigCount >= MAX_SIGNALS) return false;
  uint16_t id = nextSigId++;
  char path[24]; snprintf(path, sizeof(path), "/sig_%u.bin", id);
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  f.write(data, len); f.close();
  sigIndex[sigCount].id = id; sigIndex[sigCount].freq = freq; sigIndex[sigCount].len = len;
  sigCount++; saveIndex(); return true;
}
bool loadSignal(uint16_t id, float* freq, uint8_t* buf, uint16_t* len) {
  for (uint16_t i = 0; i < sigCount; i++) {
    if (sigIndex[i].id == id) {
      char path[24]; snprintf(path, sizeof(path), "/sig_%u.bin", id);
      File f = LittleFS.open(path, "r");
      if (!f) return false;
      *len = sigIndex[i].len; *freq = sigIndex[i].freq;
      f.read(buf, *len); f.close(); return true;
    }
  }
  return false;
}

// ==================== Radio ====================
void reinitRadio() {
  if (radio) { delete radio; radio = nullptr; }
  radio = new Si4432(new Module(PIN_CS, PIN_IRQ, PIN_SDN, PIN_GPIO));
  Serial.print("[Si4432] init... ");
  int state = radio->begin(cfgFreq, cfgBitrate, cfgFreqDev, cfgRxBw, cfgPower, cfgPreamble);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("failed, code %d\n", state);
    notifyStatus("خطای راه‌اندازی Si4432: " + String(state));
    return;
  }
  radio->setFrequency(cfgFreq);
  radio->startReceive();
  Serial.println("OK");
}

void doScan() {
  if (!scanning) return;
  if (currentFreq > scanEnd) currentFreq = scanStart;
  radio->setFrequency(currentFreq);
  radio->startReceive();
  delay(6);
  float rssi = radio->getRSSI();
  notifyScan(currentFreq, (int)rssi);
  currentFreq += scanStep;
}

void startRecording() {
  scanning = false; recording = true; recordLen = 0;
  recordFreq = currentFreq; lastRecvTime = millis();
  radio->setFrequency(recordFreq); radio->startReceive();
  notifyStatus("Recording @ " + String(recordFreq, 2) + " MHz");
}

void finishRecording() {
  recording = false;
  if (recordLen == 0) { notifyStatus("داده‌ای دریافت نشد"); return; }
  if (saveSignal(recordFreq, recordBuf, recordLen)) {
    notifyStatus("ذخیره شد: " + String(recordLen) + " بایت");
    notifySignalList();
  } else notifyStatus("حافظه پر است");
  recordLen = 0;
}

void doRecord() {
  if (!recording) return;
  if (radio->available()) {
    uint8_t buf[64];
    size_t len = radio->readData(buf, sizeof(buf));
    if (len > 0 && (recordLen + len) <= MAX_RECORD) {
      memcpy(recordBuf + recordLen, buf, len);
      recordLen += len; lastRecvTime = millis();
    }
  }
  if (recordLen > 0 && (millis() - lastRecvTime > RECORD_TIMEOUT_MS)) finishRecording();
}

void replaySignal(float freq, uint8_t* data, uint16_t len) {
  radio->setFrequency(freq);
  delay(10);
  int state = radio->transmit(data, len);
  if (state == RADIOLIB_ERR_NONE) notifyStatus("پخش شد: " + String(freq, 2) + " MHz");
  else notifyStatus("خطای پخش: " + String(state));
  delay(20); radio->startReceive();
}

// ==================== SDR ====================
unsigned long lastSDRSample = 0;
#define SDR_SAMPLE_RATE 8000
#define SDR_SAMPLES_PER_PACKET 64

void processSDR() {
  if (!cfgSDRMode) return;
  unsigned long now = millis();
  if (now - lastSDRSample < (1000 / SDR_SAMPLE_RATE) * SDR_SAMPLES_PER_PACKET) return;
  lastSDRSample = now;

  if (radio->available()) {
    uint8_t buf[64];
    size_t len = radio->readData(buf, sizeof(buf));
    if (len > 0) {
      // تبدیل بایت‌های خام به نمونه‌های صوتی نرمال‌شده
      float samples[SDR_SAMPLES_PER_PACKET];
      size_t count = min(len, (size_t)SDR_SAMPLES_PER_PACKET);
      for (size_t i = 0; i < count; i++) {
        samples[i] = ((int8_t)buf[i]) / 128.0f; // نرمال‌سازی به [-1, 1]
      }
      // ارسال به WebSocket صوتی
      audioWs.binaryAll((uint8_t*)samples, count * sizeof(float));
    }
  }
}

// ==================== WebSocket: Control ====================
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type != WS_EVT_DATA) return;
  String cmd; cmd.reserve(len + 1);
  for (size_t i = 0; i < len; i++) cmd += (char)data[i];
  cmd.trim();

  if (cmd == "SCAN") { scanning = true; recording = false; currentFreq = scanStart; notifyStatus("در حال اسکن..."); }
  else if (cmd == "RECORD") startRecording();
  else if (cmd == "STOP") { scanning = false; if (recording) finishRecording(); notifyStatus("متوقف"); }
  else if (cmd == "LIST") notifySignalList();
  else if (cmd == "GET_SETTINGS") notifySettings();
  else if (cmd.startsWith("SET_FREQ:")) {
    cfgFreq = cmd.substring(9).toFloat();
    if (cfgFreq >= 240 && cfgFreq <= 930) { radio->setFrequency(cfgFreq); notifyStatus("فرکانس: " + String(cfgFreq, 2) + " MHz"); }
  }
  else if (cmd.startsWith("REPLAY:")) {
    uint16_t id = cmd.substring(7).toInt();
    float freq; uint8_t buf[MAX_RECORD]; uint16_t blen;
    if (loadSignal(id, &freq, buf, &blen)) replaySignal(freq, buf, blen);
    else notifyStatus("سیگنال یافت نشد");
  }
  else if (cmd == "SAVE_SETTINGS") {
    // این فرمان از UI با JSON کامل می‌آید — در نسخه بعدی پیاده‌سازی می‌شود
    notifyStatus("تنظیمات ذخیره شد");
  }
  else if (cmd == "RESET_SETTINGS") {
    prefs.begin("rfcfg", false); prefs.clear(); prefs.end();
    notifyStatus("تنظیمات بازنشانی شد — لطفاً ریبوت کنید");
  }
  else if (cmd == "SDR_START") { cfgSDRMode = true; radio->startReceive(); notifyStatus("SDR فعال"); }
  else if (cmd == "SDR_STOP")  { cfgSDRMode = false; notifyStatus("SDR غیرفعال"); }
}

void onAudioWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
                    AwsEventType type, void* arg, uint8_t* data, size_t len) {
  // فقط برای اتصال کلاینت صوتی
}

// ==================== WiFi ====================
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("[WiFi] Connecting");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(500); Serial.print('.'); }
  if (WiFi.status() == WL_CONNECTED) Serial.println("\n[WiFi] IP: " + WiFi.localIP().toString());
  else {
    Serial.println("\n[WiFi] AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.println("[WiFi] AP IP: " + WiFi.softAPIP().toString());
  }
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("\n=== Si4432 Scanner Pro ===");

  if (!LittleFS.begin(true)) Serial.println("[FS] mount failed");
  loadIndex();
  loadSettings();

  initWiFi();
  reinitRadio();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  audioWs.onEvent(onAudioWsEvent);
  server.addHandler(&audioWs);

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
  audioWs.cleanupClients();

  if (scanning) doScan();
  if (recording) doRecord();
  processSDR();

  delay(1);
}
