/*
 * Si4432 BPS1EZ Universal RF Remote Replay Tool - FINAL VERSION
 * ESP32 DevKit V1 + Si4432
 * Library: nopnop2002/Arduino-SI4432
 * Features: Scan, Record (RSSI envelope), Replay, Threshold, Bitrate presets, OOK/FSK
 */

#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <si4432.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

// ==================== Pin Definitions ====================
#define PIN_CS   5
#define PIN_SDN  15
#define PIN_IRQ  16

// ==================== WiFi ====================
const char* STA_SSID = "YOUR_WIFI_SSID";
const char* STA_PASS = "YOUR_WIFI_PASSWORD";
const char* AP_SSID  = "RF-Scanner";
const char* AP_PASS  = "12345678";

// ==================== Si4432 ====================
Si4432 radio(PIN_CS, PIN_SDN, PIN_IRQ);

// ==================== Global Config ====================
Preferences prefs;
float cfgFreq = 433.92;
float cfgBitrate = 4.8;
int8_t cfgPower = 20;
int cfgRssiThreshold = -85;
float scanStart = 430.0;
float scanEnd   = 440.0;
float scanStep  = 0.1;
bool  cfgOOKMode = true;

// ==================== State ====================
bool scanning = false;
bool recording = false;
bool diagnosticMode = false;
float currentFreq = 430.0;
float recordFreq = 0;
#define MAX_RECORD 1024
uint8_t  recordBuf[MAX_RECORD];
uint16_t recordLen = 0;
unsigned long lastRecvTime = 0;
#define RECORD_TIMEOUT_MS 400

// ==================== Recording Sampling ====================
#define SAMPLE_INTERVAL_US  200
uint8_t  recordBitBuf = 0;
uint8_t  recordBitCount = 0;
unsigned long lastSampleUs = 0;

// ==================== Signal Index ====================
#define MAX_SIGNALS 16
struct SigEntry { uint16_t id; float freq; uint16_t len; uint8_t bitrate; };
SigEntry sigIndex[MAX_SIGNALS];
uint16_t sigCount = 0;
uint16_t nextSigId = 1;

// ==================== Web ====================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ==================== HTML UI ====================
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html><html lang="fa" dir="rtl"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Si4432 Pro</title><style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{font-family:Tahoma,sans-serif;background:#0a0e17;color:#e0e6ed;padding:10px;max-width:520px;margin:auto}
h1{font-size:1em;text-align:center;color:#00e5ff;margin-bottom:10px}
.card{background:#151c2c;border-radius:10px;padding:12px;margin-bottom:8px;border:1px solid #1e2a45}
.big{font-size:2.2em;font-weight:700;color:#00e5ff;font-family:monospace;text-align:center;display:block}
.unit{text-align:center;color:#7a8ba8;font-size:.85em}
.rssi-bar{height:8px;background:#1e2a45;border-radius:4px;margin-top:8px;overflow:hidden}
.rssi-fill{height:100%;width:0;background:#00e5ff;transition:width .1s}
.status{text-align:center;font-size:.8em;color:#7a8ba8;margin-top:6px}
.btn-row{display:flex;gap:6px;margin-top:4px}
button{flex:1;padding:12px 8px;border:none;border-radius:8px;font-size:.9em;font-weight:600;cursor:pointer;touch-action:manipulation}
.btn-scan{background:#00e5ff;color:#0a0e17}.btn-record{background:#ff3d71;color:#fff}
.btn-stop{background:#2d3a54;color:#e0e6ed}
.btn-replay{background:#00b894;color:#fff;padding:8px 12px;font-size:.85em;flex:0 0 auto}
.btn-del{background:#c0392b;color:#fff;padding:8px 10px;font-size:.85em;flex:0 0 auto;margin-right:4px}
.btn-diag{background:#6c5ce7;color:#fff}.btn-ook{background:#f9a825;color:#000}
.btn-preset{background:#34495e;color:#fff;padding:6px 4px;font-size:.75em;margin:0 2px}
.btn-preset.active{background:#00e5ff;color:#0a0e17}
.signal-item{padding:10px 4px;border-bottom:1px solid #1e2a45;font-size:.85em}
.signal-item:last-child{border-bottom:none}
.sig-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:6px}
.sig-info{color:#7a8ba8;font-size:.75em}
.sig-actions{display:flex;gap:4px}
.empty{color:#3a4a66;text-align:center;padding:14px}
.tabs{display:flex;gap:4px;margin-bottom:8px}
.tab{flex:1;padding:10px;text-align:center;background:#1e2a45;border-radius:6px;cursor:pointer;font-size:.85em}
.tab.active{background:#00e5ff;color:#0a0e17;font-weight:bold}
.tab-content{display:none}.tab-content.active{display:block}
.row{display:flex;gap:6px;margin-bottom:6px;align-items:center}
.label{color:#7a8ba8;font-size:.8em;min-width:65px}
input[type=number],input[type=text]{flex:1;background:#0a0e17;color:#0f0;border:1px solid #1e2a45;border-radius:6px;padding:8px;font-family:monospace;font-size:.9em}
.threshold-row{display:flex;gap:8px;align-items:center;margin-top:6px}
.threshold-row input[type=range]{flex:1}
</style></head><body>
<h1>📡 Si4432 Scanner Pro</h1>
<div class="tabs">
<div class="tab active" onclick="switchTab('scan')">اسکن</div>
<div class="tab" onclick="switchTab('settings')">تنظیمات</div>
</div>

<div id="tab-scan" class="tab-content active">
<div class="card">
<span class="big" id="freq">---.--</span><div class="unit">MHz</div>
<div class="rssi-bar"><div class="rssi-fill" id="rssiBar"></div></div>
<div class="status" id="rssiTxt">RSSI: --- dBm</div>
<div class="status" id="statusTxt">آماده</div>
</div>

<div class="card">
<div class="threshold-row">
<span class="label" style="min-width:auto">آستانه:</span>
<input type="range" id="thSlider" min="-110" max="-40" value="-85" oninput="updateTh(this.value)">
<span id="thVal" style="min-width:50px;text-align:center;color:#00e5ff;font-family:monospace">-85</span>
</div>
</div>

<div class="card">
<div class="row"><span class="label">فرکانس</span>
<input type="number" id="manualFreq" step="0.1" value="433.92" min="240" max="930">
<button class="btn-scan" onclick="setManualFreq()">تنظیم</button></div>
<div class="row"><span class="label">بیت‌ریت</span>
<button class="btn-preset" id="br24" onclick="setBitrate(2.4)">2.4</button>
<button class="btn-preset" id="br48" onclick="setBitrate(4.8)">4.8</button>
<button class="btn-preset" id="br96" onclick="setBitrate(9.6)">9.6</button>
</div>
<div class="btn-row">
<button class="btn-scan" onclick="send('SCAN')">▶ اسکن</button>
<button class="btn-record" onclick="send('RECORD')">● ضبط</button>
<button class="btn-stop" onclick="send('STOP')">■ توقف</button>
</div>
<div class="btn-row">
<button class="btn-ook" onclick="send('TOGGLE_OOK')" id="ookBtn">OOK: روشن</button>
<button class="btn-diag" onclick="send('DIAG_TOGGLE')" id="diagBtn">🔬 عیب‌یابی</button>
</div>
</div>

<div class="card">
<div style="color:#7a8ba8;font-size:.85em;margin-bottom:6px">📼 سیگنال‌های ضبط شده</div>
<div id="list"><div class="empty">خالی</div></div>
</div>
</div>

<div id="tab-settings" class="tab-content">
<div class="card"><h3 style="color:#00e5ff;font-size:.9em;margin-bottom:8px">📻 پارامترها</h3>
<div class="row"><span class="label">فرکانس</span><input type="number" id="setFreq" step="0.1" value="433.92"></div>
<div class="row"><span class="label">بیت‌ریت</span><input type="number" id="setBitrate" step="0.1" value="4.8"></div>
<div class="row"><span class="label">توان</span><input type="number" id="setPower" step="3" value="20" min="-1" max="20"></div>
<div class="row"><span class="label">آستانه</span><input type="number" id="setRssiTh" value="-85"></div>
</div>
<div class="card"><h3 style="color:#00e5ff;font-size:.9em;margin-bottom:8px">🔍 محدوده اسکن</h3>
<div class="row"><span class="label">شروع</span><input type="number" id="setScanStart" step="0.1" value="430.0"></div>
<div class="row"><span class="label">پایان</span><input type="number" id="setScanEnd" step="0.1" value="440.0"></div>
<div class="row"><span class="label">گام</span><input type="number" id="setScanStep" step="0.05" value="0.1"></div>
</div>
<div class="card">
<button class="btn-scan" onclick="saveSettings()">💾 ذخیره تنظیمات</button>
<button class="btn-stop" onclick="resetSettings()">↺ بازگشت به پیش‌فرض</button>
<button class="btn-stop" onclick="clearAllSignals()" style="margin-top:6px">🗑 پاک کردن همه سیگنال‌ها</button>
</div>
</div>

<script>
let ws;
function connectWS(){
  ws=new WebSocket('ws://'+location.host+'/ws');
  ws.onopen=()=>{setStatus('متصل');send('LIST');loadSettings();};
  ws.onclose=()=>{setStatus('قطع — تلاش مجدد');setTimeout(connectWS,2000);};
  ws.onmessage=(e)=>{try{const m=JSON.parse(e.data);
    if(m.type==='SCAN'){document.getElementById('freq').textContent=m.freq.toFixed(2);
      const pct=Math.max(0,Math.min(100,(m.rssi+110)*1.4));
      const bar=document.getElementById('rssiBar');bar.style.width=pct+'%';
      bar.style.background=m.rssi>-70?'#00e5ff':m.rssi>-85?'#f9a825':'#ff3d71';
      document.getElementById('rssiTxt').textContent='RSSI: '+m.rssi+' dBm';}
    if(m.type==='DIAG'){document.getElementById('freq').textContent=m.freq.toFixed(2);
      document.getElementById('rssiTxt').textContent='RAW: '+m.raw+' | dBm: '+m.dbm;}
    if(m.type==='SIGNALS')renderList(m.list);
    if(m.type==='STATUS')setStatus(m.msg);
    if(m.type==='SETTINGS')applySettings(m.data);
    if(m.type==='OOK'){document.getElementById('ookBtn').textContent='OOK: '+(m.on?'روشن':'خاموش');}
    if(m.type==='BITRATE'){setBitrateUI(m.value);}
  }catch(err){console.log('bad json',e.data);}};
}
function send(c){if(ws&&ws.readyState===1)ws.send(c);}
function switchTab(n){document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.querySelectorAll('.tab-content').forEach(t=>t.classList.remove('active'));
  document.querySelector('.tab[onclick*="'+n+'"]').classList.add('active');
  document.getElementById('tab-'+n).classList.add('active');}
function setManualFreq(){const f=parseFloat(document.getElementById('manualFreq').value);
  if(f>=240&&f<=930)send('SET_FREQ:'+f);}
function setBitrate(br){send('SET_BITRATE:'+br);}
function setBitrateUI(br){
  document.getElementById('br24').classList.remove('active');
  document.getElementById('br48').classList.remove('active');
  document.getElementById('br96').classList.remove('active');
  if(br==2.4)document.getElementById('br24').classList.add('active');
  else if(br==4.8)document.getElementById('br48').classList.add('active');
  else if(br==9.6)document.getElementById('br96').classList.add('active');
}
function updateTh(v){
  document.getElementById('thVal').textContent=v;
  send('THRESH:'+v);
}
function renderList(list){const el=document.getElementById('list');
  if(!list||!list.length){el.innerHTML='<div class="empty">خالی</div>';return;}
  el.innerHTML=list.map(s=>
    '<div class="signal-item">'+
    '<div class="sig-head"><div><div style="color:#00e5ff;font-family:monospace">'+s.freq.toFixed(2)+' MHz</div>'+
    '<div class="sig-info">'+s.len+' بایت • '+s.bitrate+' kbps</div></div>'+
    '<div class="sig-actions">'+
    '<button class="btn-replay" onclick="send(\'REPLAY:'+s.id+'\')">📡 پخش</button>'+
    '<button class="btn-del" onclick="delSignal('+s.id+')">🗑</button>'+
    '</div></div></div>').join('');}
function delSignal(id){if(confirm('حذف شود؟'))send('DEL:'+id);}
function clearAllSignals(){if(confirm('همه سیگنال‌ها حذف شوند؟'))send('CLEAR_ALL');}
function loadSettings(){send('GET_SETTINGS');}
function applySettings(d){document.getElementById('setFreq').value=d.freq;
  document.getElementById('setBitrate').value=d.bitrate;
  document.getElementById('setPower').value=d.power;
  document.getElementById('setRssiTh').value=d.rssiThreshold;
  document.getElementById('setScanStart').value=d.scanStart;
  document.getElementById('setScanEnd').value=d.scanEnd;
  document.getElementById('setScanStep').value=d.scanStep;
  document.getElementById('thSlider').value=d.rssiThreshold;
  document.getElementById('thVal').textContent=d.rssiThreshold;
  setBitrateUI(d.bitrate);}
function saveSettings(){const s={freq:parseFloat(document.getElementById('setFreq').value),
  bitrate:parseFloat(document.getElementById('setBitrate').value),
  power:parseInt(document.getElementById('setPower').value),
  rssiThreshold:parseInt(document.getElementById('setRssiTh').value),
  scanStart:parseFloat(document.getElementById('setScanStart').value),
  scanEnd:parseFloat(document.getElementById('setScanEnd').value),
  scanStep:parseFloat(document.getElementById('setScanStep').value)};
  send('SAVE_SETTINGS:'+JSON.stringify(s));}
function resetSettings(){if(confirm('بازگشت به پیش‌فرض؟'))send('RESET_SETTINGS');}
function setStatus(s){document.getElementById('statusTxt').textContent=s;}
connectWS();
</script></body></html>
)HTMLPAGE";

// ==================== Forward Declarations ====================
void finishRecording();
void reinitRadio();
void loadSettings();
void saveSettingsToPrefs();
byte readRssiRegister();
int readRSSI();

// ==================== JSON Helpers ====================
void notifyScan(float freq, int rssi) {
  ws.textAll("{\"type\":\"SCAN\",\"freq\":" + String(freq, 2) + ",\"rssi\":" + String(rssi) + "}");
}
void notifyDiag(float freq, int raw, int dbm) {
  ws.textAll("{\"type\":\"DIAG\",\"freq\":" + String(freq, 2) + ",\"raw\":" + String(raw) + ",\"dbm\":" + String(dbm) + "}");
}
void notifyStatus(const String& msg) {
  String s = msg; s.replace("\\", "\\\\"); s.replace("\"", "\\\"");
  ws.textAll("{\"type\":\"STATUS\",\"msg\":\"" + s + "\"}");
}
void notifySignalList() {
  String out = "{\"type\":\"SIGNALS\",\"list\":[";
  for (uint16_t i = 0; i < sigCount; i++) {
    if (i) out += ",";
    out += "{\"id\":" + String(sigIndex[i].id) + ",\"freq\":" + String(sigIndex[i].freq, 2) + ",\"len\":" + String(sigIndex[i].len) + ",\"bitrate\":" + String(sigIndex[i].bitrate, 1) + "}";
  }
  out += "]}"; ws.textAll(out);
}
void notifySettings() {
  String out = "{\"type\":\"SETTINGS\",\"data\":{";
  out += "\"freq\":" + String(cfgFreq, 2);
  out += ",\"bitrate\":" + String(cfgBitrate, 2);
  out += ",\"power\":" + String(cfgPower);
  out += ",\"rssiThreshold\":" + String(cfgRssiThreshold);
  out += ",\"scanStart\":" + String(scanStart, 2);
  out += ",\"scanEnd\":" + String(scanEnd, 2);
  out += ",\"scanStep\":" + String(scanStep, 2);
  out += "}}"; ws.textAll(out);
}

// ==================== Preferences ====================
void loadSettings() {
  prefs.begin("rfcfg", true);
  cfgFreq = prefs.getFloat("freq", 433.92);
  cfgBitrate = prefs.getFloat("bitrate", 4.8);
  cfgPower = prefs.getInt("power", 20);
  cfgRssiThreshold = prefs.getInt("rssiTh", -85);
  scanStart = prefs.getFloat("scanStart", 430.0);
  scanEnd = prefs.getFloat("scanEnd", 440.0);
  scanStep = prefs.getFloat("scanStep", 0.1);
  cfgOOKMode = prefs.getBool("ook", true);
  prefs.end();
}
void saveSettingsToPrefs() {
  prefs.begin("rfcfg", false);
  prefs.putFloat("freq", cfgFreq); prefs.putFloat("bitrate", cfgBitrate);
  prefs.putInt("power", cfgPower);
  prefs.putInt("rssiTh", cfgRssiThreshold);
  prefs.putFloat("scanStart", scanStart); prefs.putFloat("scanEnd", scanEnd);
  prefs.putFloat("scanStep", scanStep); prefs.putBool("ook", cfgOOKMode);
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
  File f = LittleFS.open("/idx.bin", "w"); if (!f) return;
  f.write((uint8_t*)&sigCount, sizeof(sigCount));
  for (uint16_t i = 0; i < sigCount; i++) f.write((uint8_t*)&sigIndex[i], sizeof(SigEntry));
  f.close();
}
bool saveSignal(float freq, uint8_t* data, uint16_t len, uint8_t bitrate) {
  if (sigCount >= MAX_SIGNALS) return false;
  uint16_t id = nextSigId++;
  char path[24]; snprintf(path, sizeof(path), "/sig_%u.bin", id);
  File f = LittleFS.open(path, "w"); if (!f) return false;
  f.write(data, len); f.close();
  sigIndex[sigCount].id = id; sigIndex[sigCount].freq = freq;
  sigIndex[sigCount].len = len; sigIndex[sigCount].bitrate = bitrate;
  sigCount++; saveIndex(); return true;
}
bool loadSignal(uint16_t id, float* freq, uint8_t* buf, uint16_t* len, uint8_t* br) {
  for (uint16_t i = 0; i < sigCount; i++) {
    if (sigIndex[i].id == id) {
      char path[24]; snprintf(path, sizeof(path), "/sig_%u.bin", id);
      File f = LittleFS.open(path, "r"); if (!f) return false;
      *len = sigIndex[i].len; *freq = sigIndex[i].freq; *br = sigIndex[i].bitrate;
      f.read(buf, *len); f.close(); return true;
    }
  }
  return false;
}
void deleteSignal(uint16_t id) {
  for (uint16_t i = 0; i < sigCount; i++) {
    if (sigIndex[i].id == id) {
      char path[24]; snprintf(path, sizeof(path), "/sig_%u.bin", id);
      LittleFS.remove(path);
      for (uint16_t j = i; j < sigCount - 1; j++) sigIndex[j] = sigIndex[j + 1];
      sigCount--; saveIndex(); return;
    }
  }
}
void clearAllSignals() {
  for (uint16_t i = 0; i < sigCount; i++) {
    char path[24]; snprintf(path, sizeof(path), "/sig_%u.bin", sigIndex[i].id);
    LittleFS.remove(path);
  }
  sigCount = 0; saveIndex();
}

// ==================== Radio Init ====================
void reinitRadio() {
  Serial.print("[Si4432] init... ");
  if (!radio.init()) {
    Serial.println("FAILED");
    notifyStatus("خطای راه‌اندازی Si4432");
    return;
  }
  radio.setFrequency(cfgFreq);
  radio.setBaudRate(cfgBitrate);
  radio.setModulationType(cfgOOKMode ? Si4432::OOK : Si4432::GFSK);
  radio.setTransmitPower((byte)map(cfgPower, -1, 20, 0, 7));
  radio.setPacketHandling(false);
  radio.setManchesterEncoding(false);
  radio.turnOn();
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  Serial.println("OK (OOK=" + String(cfgOOKMode) + ", BR=" + String(cfgBitrate) + ")");
}

// ==================== RSSI Direct SPI ====================
byte readRssiRegister() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(0x26 & 0x7F);
  byte result = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return result;
}

int readRSSI() {
  byte raw = readRssiRegister();
  int dbm = (int)(0.5 * raw) - 131;
  return dbm;
}

// ==================== Scan ====================
void doScan() {
  if (!scanning) return;
  if (currentFreq > scanEnd) currentFreq = scanStart;

  radio.setFrequency(currentFreq);
  radio.turnOn();
  delay(30);

  int rssi = readRSSI();
  notifyScan(currentFreq, rssi);

  currentFreq += scanStep;
}

// ==================== Diagnostic ====================
void doDiagnostic() {
  if (!diagnosticMode) return;

  radio.setFrequency(currentFreq);
  radio.turnOn();
  delay(30);

  byte raw = readRssiRegister();
  int rssi = (int)(0.5 * raw) - 131;
  notifyDiag(currentFreq, raw, rssi);

  Serial.print("[DIAG] ");
  Serial.print(currentFreq, 2);
  Serial.print(" MHz | RAW=");
  Serial.print(raw);
  Serial.print(" | dBm=");
  Serial.println(rssi);

  currentFreq += scanStep;
  if (currentFreq > scanEnd) currentFreq = scanStart;
}

// ==================== Recording (RSSI envelope) ====================
void startRecording() {
  scanning = false; recording = true; diagnosticMode = false;
  recordLen = 0; recordBitBuf = 0; recordBitCount = 0;
  recordFreq = currentFreq;
  lastRecvTime = millis();
  lastSampleUs = micros();
  radio.setFrequency(recordFreq);
  radio.turnOn();
  delay(50);
  notifyStatus("Recording @ " + String(recordFreq, 2) + " MHz");
}

void finishRecording() {
  recording = false;
  if (recordLen == 0) { notifyStatus("داده‌ای دریافت نشد"); return; }
  if (saveSignal(recordFreq, recordBuf, recordLen, (uint8_t)cfgBitrate)) {
    notifyStatus("✅ ذخیره شد: " + String(recordLen) + " بایت");
    notifySignalList();
  } else notifyStatus("❌ حافظه پر است");
  recordLen = 0;
}

void doRecord() {
  if (!recording) return;

  unsigned long nowUs = micros();
  if (nowUs - lastSampleUs < SAMPLE_INTERVAL_US) return;
  lastSampleUs = nowUs;

  byte raw = readRssiRegister();
  int dbm = (int)(0.5 * raw) - 131;
  bool signal = (dbm > cfgRssiThreshold);

  if (signal) recordBitBuf |= (0x80 >> recordBitCount);
  recordBitCount++;

  if (recordBitCount == 8) {
    if (recordLen < MAX_RECORD) recordBuf[recordLen++] = recordBitBuf;
    recordBitBuf = 0;
    recordBitCount = 0;
    if (signal) lastRecvTime = millis();
  }

  if (recordLen > 2 && (millis() - lastRecvTime > RECORD_TIMEOUT_MS)) {
    if (recordBitCount > 0 && recordLen < MAX_RECORD) recordBuf[recordLen++] = recordBitBuf;
    finishRecording();
  }
}

// ==================== Replay ====================
void replaySignal(float freq, uint8_t* data, uint16_t len, uint8_t br) {
  float savedBr = cfgBitrate;
  cfgBitrate = br;
  radio.setBaudRate(cfgBitrate);
  radio.setFrequency(freq);
  radio.turnOn();
  delay(20);
  radio.sendPacket((uint8_t)len, data);
  delay(50);
  cfgBitrate = savedBr;
  radio.setBaudRate(cfgBitrate);
  radio.startListening();
  notifyStatus("📡 پخش شد: " + String(freq, 2) + " MHz");
}

// ==================== WebSocket ====================
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type != WS_EVT_DATA) return;
  String cmd; cmd.reserve(len + 1);
  for (size_t i = 0; i < len; i++) cmd += (char)data[i];
  cmd.trim();

  if (cmd == "SCAN") { scanning = true; recording = false; diagnosticMode = false; currentFreq = scanStart; notifyStatus("در حال اسکن..."); }
  else if (cmd == "RECORD") startRecording();
  else if (cmd == "STOP") { scanning = false; if (recording) finishRecording(); diagnosticMode = false; notifyStatus("متوقف"); }
  else if (cmd == "LIST") notifySignalList();
  else if (cmd == "GET_SETTINGS") notifySettings();
  else if (cmd == "CLEAR_ALL") { clearAllSignals(); notifySignalList(); notifyStatus("همه حذف شد"); }
  else if (cmd.startsWith("DEL:")) {
    uint16_t id = cmd.substring(4).toInt();
    deleteSignal(id); notifySignalList(); notifyStatus("حذف شد");
  }
  else if (cmd == "DIAG_TOGGLE") {
    diagnosticMode = !diagnosticMode; scanning = false; recording = false;
    if (diagnosticMode) { currentFreq = scanStart; notifyStatus("حالت عیب‌یابی فعال"); }
    else notifyStatus("حالت عیب‌یابی غیرفعال");
  }
  else if (cmd == "TOGGLE_OOK") {
    cfgOOKMode = !cfgOOKMode;
    saveSettingsToPrefs();
    reinitRadio();
    ws.textAll("{\"type\":\"OOK\",\"on\":" + String(cfgOOKMode ? "true" : "false") + "}");
    notifyStatus(cfgOOKMode ? "OOK mode ON" : "GFSK mode ON");
  }
  else if (cmd.startsWith("SET_BITRATE:")) {
    float br = cmd.substring(12).toFloat();
    if (br >= 0.5 && br <= 128) {
      cfgBitrate = br;
      radio.setBaudRate(cfgBitrate);
      saveSettingsToPrefs();
      ws.textAll("{\"type\":\"BITRATE\",\"value\":" + String(br, 1) + "}");
      notifyStatus("بیت‌ریت: " + String(br, 1) + " kbps");
    }
  }
  else if (cmd.startsWith("THRESH:")) {
    cfgRssiThreshold = cmd.substring(7).toInt();
    saveSettingsToPrefs();
  }
  else if (cmd.startsWith("SET_FREQ:")) {
    cfgFreq = cmd.substring(9).toFloat();
    if (cfgFreq >= 240 && cfgFreq <= 930) { radio.setFrequency(cfgFreq); notifyStatus("فرکانس: " + String(cfgFreq, 2) + " MHz"); }
  }
  else if (cmd.startsWith("REPLAY:")) {
    uint16_t id = cmd.substring(7).toInt();
    float freq; uint8_t buf[MAX_RECORD]; uint16_t blen; uint8_t br;
    if (loadSignal(id, &freq, buf, &blen, &br)) replaySignal(freq, buf, blen, br);
    else notifyStatus("سیگنال یافت نشد");
  }
  else if (cmd.startsWith("SAVE_SETTINGS:")) {
    // parse JSON manually (simple)
    int p;
    p = cmd.indexOf("\"freq\":"); if (p > 0) cfgFreq = cmd.substring(p + 7).toFloat();
    p = cmd.indexOf("\"bitrate\":"); if (p > 0) cfgBitrate = cmd.substring(p + 10).toFloat();
    p = cmd.indexOf("\"power\":"); if (p > 0) cfgPower = cmd.substring(p + 8).toInt();
    p = cmd.indexOf("\"rssiThreshold\":"); if (p > 0) cfgRssiThreshold = cmd.substring(p + 16).toInt();
    p = cmd.indexOf("\"scanStart\":"); if (p > 0) scanStart = cmd.substring(p + 12).toFloat();
    p = cmd.indexOf("\"scanEnd\":"); if (p > 0) scanEnd = cmd.substring(p + 10).toFloat();
    p = cmd.indexOf("\"scanStep\":"); if (p > 0) scanStep = cmd.substring(p + 11).toFloat();
    saveSettingsToPrefs();
    reinitRadio();
    notifyStatus("✅ تنظیمات ذخیره شد");
  }
  else if (cmd == "RESET_SETTINGS") {
    prefs.begin("rfcfg", false); prefs.clear(); prefs.end();
    notifyStatus("تنظیمات بازنشانی شد — ریبوت کنید");
  }
}

// ==================== WiFi / Setup / Loop ====================
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("[WiFi] Connecting");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(500); Serial.print('.'); }
  if (WiFi.status() == WL_CONNECTED) Serial.println("\n[WiFi] IP: " + WiFi.localIP().toString());
  else {
    Serial.println("\n[WiFi] AP mode");
    WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID, AP_PASS);
    Serial.println("[WiFi] AP IP: " + WiFi.softAPIP().toString());
  }
}
void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("\n=== Si4432 B1 Scanner Pro ===");
  if (!LittleFS.begin(true)) Serial.println("[FS] mount failed");
  loadIndex(); loadSettings();
  initWiFi();
  reinitRadio();

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
void loop() {
  ElegantOTA.loop();
  ws.cleanupClients();
  if (diagnosticMode) doDiagnostic();
  else if (scanning) doScan();
  if (recording) doRecord();
  delay(1);
}
