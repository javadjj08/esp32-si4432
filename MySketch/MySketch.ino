/*
 * Si4432 Pro - Ultimate RF Analysis & Security Tool
 * ESP32 DevKit V1 + Si4432
 * Library: nopnop2002/Arduino-SI4432
 */

#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <si4432.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

// ==================== Pins ====================
#define PIN_CS   5
#define PIN_SDN  15
#define PIN_IRQ  16

// ==================== WiFi ====================
const char* AP_SSID = "RF-Recorder";
const char* AP_PASS = "12345678";

// ==================== Radio ====================
Si4432 radio(PIN_CS, PIN_SDN, PIN_IRQ);
Preferences prefs;

float cfgFreq = 433.92;
float cfgBitrate = 4.8;
int   cfgPower = 20;
int   cfgThreshold = -85;
bool  cfgOOK = true;
bool  cfgManchester = false;

// ==================== State ====================
bool scanning = false;
bool sdrActive = false;
bool diagMode = false;
int  recState = 0;
float currentFreq = 433.92;
float scanStart = 300.0;
float scanEnd   = 928.0;
float scanStep  = 0.5;

// Adaptive Scan
bool adaptiveScanEnabled = true;
int   adaptiveThreshold = -85;
int   noiseFloor = -100;
int   noiseSamples = 0;
long  noiseSum = 0;

// Smart Scan
bool  smartScanEnabled = true;
float smartScanCoarseStep = 1.0;
float smartScanFineStep = 0.05;
int   smartScanThreshold = -90;
float lastStrongFreq = 0;
int   smartScanPhase = 0;
float smartScanFreq = 0;

// Signal Tracker
bool trackerEnabled = false;
float trackerFreq = 0;
unsigned long lastTrackerUpdate = 0;
#define TRACKER_INTERVAL 500

// Frequency Hopping
bool hoppingEnabled = false;
const float hopFreqs[] = {433.05, 433.42, 433.92, 434.42, 868.35, 915.00};
int hopIndex = 0;
unsigned long lastHopTime = 0;
#define HOP_INTERVAL 250

// Sensors
float chipTemp = 0;
float supplyVoltage = 0;

// Recording
float recFreq = 0;
uint8_t  recBuf[2048];
uint16_t recLen = 0;
uint8_t  recBit = 0;
uint8_t  recBitCnt = 0;
unsigned long recStartMs = 0;
unsigned long recLastMs = 0;
char     recName[32] = "";
char     recNote[64] = "";
char     recCategory[24] = "Default";
char     recTags[64] = "";

#define REC_TIMEOUT_MS 300
#define REC_START_TIMEOUT_MS 5000

// SDR
#define SDR_BUF_SIZE 128
uint8_t sdrBuffer[SDR_BUF_SIZE];
int sdrBufIdx = 0;
unsigned long lastSdrSample = 0;
#define SDR_SAMPLE_INTERVAL_US 125

// Diagnostic
int diagStep = 0;
bool diagComplete = false;

// Signals
#define MAX_SIGNALS 64
#define NAME_LEN    32
struct Signal {
  uint16_t id;
  float freq;
  uint16_t len;
  char name[NAME_LEN];
  char category[24];
  char tags[64];
};
Signal signals[MAX_SIGNALS];
int signalCount = 0;
int nextId = 1;

// Webhook
bool webhookEnabled = false;
char webhookUrl[128] = "";

// Web
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocket sdrSocket("/sdr");

// ==================== HTML ====================
const char HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="fa" dir="rtl"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Si4432 Ultimate</title><style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;user-select:none}
body{font-family:Tahoma,sans-serif;background:#0a0e17;color:#e0e6ed;
     max-width:520px;margin:auto;min-height:100vh;display:flex;flex-direction:column}
#statusbar{background:#0d1420;padding:8px 12px;display:flex;justify-content:space-between;
  align-items:center;border-bottom:1px solid #1e2a45;font-family:monospace;font-size:.8em;
  min-height:40px;position:sticky;top:0;z-index:10}
#sbFreq{color:#00e5ff;font-weight:bold}#sbRssi{color:#f9a825}#sbState{color:#8899aa}
.screen{display:none;padding:12px;flex:1;overflow-y:auto}
.screen.active{display:block}
.header{display:flex;align-items:center;gap:10px;margin-bottom:12px;
  padding-bottom:8px;border-bottom:1px solid #1e2a45}
.back{background:transparent;color:#00e5ff;border:none;font-size:1.4em;
  cursor:pointer;padding:4px 12px;flex:0 0 auto}
.title{font-size:1.05em;font-weight:bold;color:#e0e6ed;flex:1}
.menu{display:grid;grid-template-columns:1fr 1fr;gap:10px;padding:8px 0}
.menu-item{background:#151c2c;border:1px solid #1e2a45;border-radius:12px;
  padding:18px 8px;display:flex;flex-direction:column;align-items:center;
  justify-content:center;gap:8px;cursor:pointer;transition:all .15s;min-height:110px}
.menu-item:active{transform:scale(.96);background:#1e2a45}
.menu-icon{font-size:2.2em;line-height:1}
.menu-label{font-size:.8em;color:#e0e6ed;text-align:center;font-weight:600}
.menu-sub{font-size:.65em;color:#7a8ba8;text-align:center;margin-top:2px}
.card{background:#151c2c;border-radius:10px;padding:12px;margin-bottom:10px;border:1px solid #1e2a45}
.row{display:flex;gap:6px;margin-bottom:8px;align-items:center;flex-wrap:wrap}
.label{color:#7a8ba8;font-size:.8em;min-width:65px}
input[type=number],input[type=text],input[type=url],select{flex:1;background:#0a0e17;color:#0f0;
  border:1px solid #1e2a45;border-radius:6px;padding:10px;font-family:monospace;
  font-size:.9em;color-scheme:dark;min-width:100px}
input:focus{outline:none;border-color:#00e5ff}
.btn{padding:12px;border:none;border-radius:8px;font-size:.9em;font-weight:600;
  cursor:pointer;touch-action:manipulation;font-family:inherit}
.btn:active{transform:scale(.97)}
.btn-primary{background:#00e5ff;color:#0a0e17;width:100%}
.btn-danger{background:#ff3d71;color:#fff;flex:1}
.btn-stop{background:#2d3a54;color:#e0e6ed;flex:1}
.btn-green{background:#00b894;color:#fff}
.btn-ook{background:#f9a825;color:#000;flex:1}
.btn-preset{background:#34495e;color:#fff;padding:8px 4px;font-size:.7em;flex:1;min-width:50px;margin:2px}
.btn-preset.active{background:#00e5ff;color:#0a0e17}
.btn-row{display:flex;gap:6px;margin-top:6px;flex-wrap:wrap}
.presets{display:flex;flex-wrap:wrap;gap:4px;margin-top:6px}
.display{background:#0a0e17;border-radius:10px;padding:20px;text-align:center;
  margin-bottom:10px;border:1px solid #1e2a45}
.display .value{font-size:2.4em;font-weight:700;color:#00e5ff;
  font-family:monospace;display:block;line-height:1.1}
.display .unit{color:#7a8ba8;font-size:.9em;margin-top:4px}
.rssi-bar{height:8px;background:#1e2a45;border-radius:4px;margin-top:12px;overflow:hidden}
.rssi-fill{height:100%;width:0;background:#00e5ff;transition:width .15s}
.hint{text-align:center;color:#8899aa;font-size:.8em;margin-top:8px;font-family:monospace}
.sig{background:#0a0e17;padding:12px;border-radius:8px;margin-bottom:8px;border:1px solid #1e2a45}
.sig-head{display:flex;justify-content:space-between;align-items:flex-start}
.sig-name{color:#00e5ff;font-weight:bold;font-size:.95em;word-break:break-word}
.sig-meta{color:#8899aa;font-size:.75em;font-family:monospace;margin-top:4px}
.sig-note{color:#f9a825;font-size:.75em;margin-top:3px;font-style:italic}
.sig-category{color:#6c5ce7;font-size:.7em;margin-top:2px}
.sig-tags{color:#00b894;font-size:.7em;margin-top:2px}
.sig-actions{display:flex;gap:4px;flex:0 0 auto;flex-wrap:wrap}
.empty{color:#3a4a66;text-align:center;padding:30px 10px;font-size:.85em}
.rec-overlay{position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(10,14,23,.95);
  display:none;flex-direction:column;align-items:center;justify-content:center;
  padding:20px;z-index:100}
.rec-overlay.active{display:flex}
.rec-pulse{width:80px;height:80px;background:#ff3d71;border-radius:50%;
  animation:pulse 1.2s infinite;margin-bottom:20px}
@keyframes pulse{0%{transform:scale(1);opacity:1}50%{transform:scale(1.15);opacity:.6}100%{transform:scale(1);opacity:1}}
.rec-rssi{font-size:2em;color:#00e5ff;font-family:monospace;font-weight:bold}
.rec-status{color:#e0e6ed;margin-top:12px;font-size:1em;text-align:center}
.rec-cancel{margin-top:24px;background:#2d3a54;color:#fff;padding:12px 40px;
  border:none;border-radius:8px;font-size:1em;font-family:inherit}
input[type=range]{flex:1;height:8px;-webkit-appearance:none;background:#1e2a45;
  border-radius:4px;outline:none;min-width:120px}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;
  border-radius:50%;background:#00e5ff;cursor:pointer}
.th-val{min-width:55px;text-align:center;color:#00e5ff;font-family:monospace;font-weight:bold}
.diag-box{background:#0a0e17;padding:15px;border-radius:8px;margin-top:10px;font-family:monospace;font-size:.85em;line-height:1.6}
.diag-ok{color:#00b894}.diag-fail{color:#ff3d71}.diag-warn{color:#f9a825}
#rssiGraph{width:100%;height:80px;background:#0a0e17;border-radius:8px;margin-top:10px}
.modal{display:none;position:fixed;top:0;left:0;right:0;bottom:0;
  background:rgba(0,0,0,.85);z-index:200;align-items:center;justify-content:center;padding:20px}
.modal.active{display:flex}
.modal-content{background:#151c2c;padding:20px;border-radius:12px;
  max-width:500px;width:100%;max-height:80vh;overflow-y:auto}
.hexbox{font-family:monospace;font-size:.75em;background:#0a0e17;padding:12px;
  border-radius:6px;word-break:break-all;line-height:1.6;color:#0f0}
</style></head><body>

<div id="statusbar">
  <span id="sbFreq">433.92 MHz</span>
  <span id="sbRssi">-- dBm</span>
  <span id="sbState">● idle</span>
</div>

<div id="sc-home" class="screen active">
  <div class="header"><div class="title">📡 Si4432 Ultimate</div></div>
  <div class="menu">
    <div class="menu-item" onclick="go('read')"><span class="menu-icon">📻</span><span class="menu-label">Read</span><span class="menu-sub">ضبط سیگنال</span></div>
    <div class="menu-item" onclick="go('saved')"><span class="menu-icon">📼</span><span class="menu-label">Saved</span><span class="menu-sub">مدیریت</span></div>
    <div class="menu-item" onclick="go('analyzer')"><span class="menu-icon">🔍</span><span class="menu-label">Analyzer</span><span class="menu-sub">اسکن هوشمند</span></div>
    <div class="menu-item" onclick="go('sdr')"><span class="menu-icon">🔊</span><span class="menu-label">SDR</span><span class="menu-sub">شنیدن صدا</span></div>
    <div class="menu-item" onclick="go('sensors')"><span class="menu-icon">🌡️</span><span class="menu-label">Sensors</span><span class="menu-sub">دما/ولتاژ</span></div>
    <div class="menu-item" onclick="go('hopping')"><span class="menu-icon">📡</span><span class="menu-label">Hopping</span><span class="menu-sub">پرش فرکانسی</span></div>
    <div class="menu-item" onclick="go('attack')"><span class="menu-icon">🎯</span><span class="menu-label">Attack</span><span class="menu-sub">شبیه‌ساز</span></div>
    <div class="menu-item" onclick="go('diagnostic')"><span class="menu-icon">🧪</span><span class="menu-label">Diagnostic</span><span class="menu-sub">عیب‌یابی</span></div>
    <div class="menu-item" onclick="go('settings')"><span class="menu-icon">⚙️</span><span class="menu-label">Settings</span><span class="menu-sub">تنظیمات</span></div>
  </div>
</div>

<div id="sc-read" class="screen">
  <div class="header"><button class="back" onclick="go('home')">←</button><div class="title">📻 Read Signal</div></div>
  <div class="card">
    <div class="row"><span class="label">اسم</span><input type="text" id="recName" placeholder="درب پارکینگ" maxlength="30"></div>
    <div class="row"><span class="label">دسته</span><input type="text" id="recCategory" placeholder="پارکینگ" maxlength="20"></div>
    <div class="row"><span class="label">برچسب‌ها</span><input type="text" id="recTags" placeholder="ورودی, اصلی" maxlength="60"></div>
    <div class="row"><span class="label">یادداشت</span><input type="text" id="recNoteIn" placeholder="اختیاری" maxlength="60"></div>
    <div class="row"><span class="label">فرکانس</span><input type="number" id="recFreq" step="0.01" value="433.92" min="240" max="930"></div>
    <div class="row"><span class="label">بیت‌ریت</span>
      <button class="btn-preset" id="rbr24" onclick="setBitrate(2.4)">2.4</button>
      <button class="btn-preset active" id="rbr48" onclick="setBitrate(4.8)">4.8</button>
      <button class="btn-preset" id="rbr96" onclick="setBitrate(9.6)">9.6</button>
    </div>
    <div class="row"><span class="label">Manchester</span><button class="btn-preset" id="manch" onclick="toggleManchester()">Manchester: خاموش</button></div>
    <div class="row"><span class="label">آستانه</span>
      <input type="range" id="recTh" min="-110" max="-40" value="-85" oninput="document.getElementById('recThV').textContent=this.value">
      <span class="th-val" id="recThV">-85</span>
    </div>
    <div class="btn-row"><button class="btn btn-ook" style="flex:1" onclick="autoThreshold()">⚙️ کالیبراسیون خودکار</button></div>
    <button class="btn btn-primary" style="margin-top:10px" onclick="beginRecord()">🔴 شروع ضبط</button>
    <div class="hint">ریموت را ۲-۳ سانتی‌متر از آنتن فشار بده</div>
  </div>
</div>

<div id="sc-saved" class="screen">
  <div class="header"><button class="back" onclick="go('home')">←</button><div class="title">📼 Saved Signals</div>
    <button class="btn btn-stop" style="flex:0 0 auto;padding:6px 10px;font-size:.8em" onclick="clearAll()">🗑 همه</button></div>
  <div class="card"><div class="row"><span class="label">جستجو</span>
    <input type="text" id="searchInput" placeholder="نام یا برچسب..." oninput="filterSignals()"></div></div>
  <div id="savedList"><div class="empty">خالی</div></div>
</div>

<div id="sc-analyzer" class="screen">
  <div class="header"><button class="back" onclick="go('home')">←</button><div class="title">🔍 Analyzer</div></div>
  <div class="display">
    <span class="value" id="anFreq">---.--</span><div class="unit">MHz</div>
    <div class="rssi-bar"><div class="rssi-fill" id="anBar"></div></div>
    <div class="hint" id="anRssi">RSSI: --- dBm</div>
    <div class="hint" id="anNoise">Noise: --- dBm</div>
    <canvas id="rssiGraph"></canvas>
    <div class="hint">Spectrum (60 samples)</div>
  </div>
  <div class="card"><div style="color:#00e5ff;font-size:.9em;margin-bottom:8px">📡 Preset Frequencies</div>
    <div class="presets" id="presetList"></div></div>
  <div class="card">
    <div class="row"><label><input type="checkbox" id="adaptiveScanCheck" checked onchange="toggleAdaptiveScan()"> Adaptive Scan</label></div>
    <div class="row"><span class="label">شروع</span><input type="number" id="anStart" step="0.5" value="300.0"></div>
    <div class="row"><span class="label">پایان</span><input type="number" id="anEnd" step="0.5" value="928.0"></div>
    <div class="row"><span class="label">گام</span><input type="number" id="anStep" step="0.05" value="0.5"></div>
    <div class="btn-row">
      <button class="btn btn-primary" id="anBtn" onclick="toggleScan()">▶ شروع اسکن</button>
      <button class="btn btn-green" id="trackerBtn" onclick="toggleTracker()">🔒 ردیاب</button>
    </div>
  </div>
  <div class="card">
    <div class="row"><span class="label">قفل روی</span><input type="number" id="anTarget" step="0.01" value="433.92"></div>
    <button class="btn btn-green" style="width:100%;margin-top:6px" onclick="lockFreq()">🔒 قفل و رفتن به ضبط</button>
  </div>
</div>

<div id="sc-sdr" class="screen">
  <div class="header"><button class="back" onclick="go('home')">←</button><div class="title">🔊 SDR</div></div>
  <div class="display"><span class="value" id="sdrFreq">---.--</span><div class="unit">MHz</div>
    <div class="hint" id="sdrStatus">متوقف</div></div>
  <div class="card">
    <div class="row"><span class="label">فرکانس</span>
      <input type="number" id="sdrFreqInput" step="0.01" value="433.92" min="240" max="930">
      <button class="btn btn-green" style="flex:0 0 auto;padding:10px 14px" onclick="setSdrFreq()">تنظیم</button></div>
    <button class="btn btn-primary" id="sdrBtn" onclick="toggleSdr()" style="margin-top:8px">▶ پخش صدا</button>
  </div>
  <div class="card"><button class="btn btn-ook" style="width:100%" onclick="send('SDR_SCAN')">🔍 پیدا کردن سیگنال صوتی</button></div>
</div>

<div id="sc-sensors" class="screen">
  <div class="header"><button class="back" onclick="go('home')">←</button><div class="title">🌡️ Sensors</div></div>
  <div class="card" style="text-align:center"><div style="font-size:3em;color:#00e5ff;font-family:monospace" id="tempDisplay">--</div>
    <div style="color:#7a8ba8;font-size:.9em">دمای تراشه (°C)</div></div>
  <div class="card" style="text-align:center"><div style="font-size:3em;color:#00b894;font-family:monospace" id="voltDisplay">--</div>
    <div style="color:#7a8ba8;font-size:.9em">ولتاژ تغذیه (V)</div></div>
  <button class="btn btn-primary" onclick="readSensors()">🔄 به‌روزرسانی</button>
</div>

<div id="sc-hopping" class="screen">
  <div class="header"><button class="back" onclick="go('home')">←</button><div class="title">📡 Frequency Hopping</div></div>
  <div class="card">
    <div class="row"><span class="label">وضعیت</span>
      <button class="btn btn-ook" id="hopBtn" onclick="toggleHopping()">📡 پرش فرکانسی: خاموش</button></div>
    <div class="hint">فرکانس‌های زیر به صورت دوره‌ای اسکن می‌شوند:</div>
    <div class="hint" id="hopList">433.05, 433.42, 433.92, 434.42, 868.35, 915.00 MHz</div>
  </div>
</div>

<div id="sc-attack" class="screen">
  <div class="header"><button class="back" onclick="go('home')">←</button><div class="title">🎯 Attack Simulator</div></div>
  <div class="card">
    <div style="color:#00e5ff;font-size:.9em;margin-bottom:8px">📡 Replay Attack</div>
    <div class="row"><span class="label">ID</span><input type="number" id="attackId" value="1" min="1"></div>
    <div class="row"><span class="label">تعداد</span><input type="number" id="attackCount" value="5" min="1" max="20"></div>
    <button class="btn btn-primary" onclick="replayAttack()">🔴 اجرای Replay</button>
  </div>
  <div class="card">
    <div style="color:#00e5ff;font-size:.9em;margin-bottom:8px">💥 Brute Force</div>
    <div class="row"><span class="label">فرکانس</span><input type="number" id="bruteFreq" step="0.01" value="433.92"></div>
    <div class="row"><span class="label">تعداد</span><input type="number" id="bruteCount" value="50" min="1" max="500"></div>
    <button class="btn btn-ook" onclick="bruteForce()">💥 اجرای Brute Force</button>
  </div>
</div>

<div id="sc-diagnostic" class="screen">
  <div class="header"><button class="back" onclick="go('home')">←</button><div class="title">🧪 Diagnostic</div></div>
  <div class="card">
    <button class="btn btn-primary" onclick="startDiagnostic()">▶ تست جامع</button>
    <div class="diag-box" id="diagOutput"><div>آماده...</div></div>
  </div>
  <div class="card">
    <button class="btn btn-ook" style="width:100%" onclick="send('DIAG_SPI')">📡 تست SPI</button>
    <div class="diag-box" id="spiOutput"><div>در انتظار...</div></div>
  </div>
</div>

<div id="sc-settings" class="screen">
  <div class="header"><button class="back" onclick="go('home')">←</button><div class="title">⚙️ Settings</div></div>
  <div class="card">
    <div class="row"><span class="label">Webhook</span>
      <label><input type="checkbox" id="webhookCheck" onchange="toggleWebhook()"> فعال</label></div>
    <div class="row"><span class="label">URL</span>
      <input type="url" id="webhookUrl" placeholder="http://..." maxlength="120"></div>
    <button class="btn btn-primary" onclick="saveWebhook()">💾 ذخیره Webhook</button>
  </div>
</div>

<div class="rec-overlay" id="recOverlay">
  <div class="rec-pulse"></div>
  <div class="rec-rssi" id="recLiveRssi">--- dBm</div>
  <div class="rec-status" id="recStatus">آماده ضبط</div>
  <button class="rec-cancel" onclick="cancelRecord()">لغو</button>
</div>

<div class="modal" id="hexModal">
  <div class="modal-content">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:12px">
      <div style="color:#00e5ff;font-weight:bold" id="hexTitle">Hex View</div>
      <button class="btn btn-stop" style="flex:0 0 auto;padding:6px 12px" onclick="closeHex()">بستن</button>
    </div>
    <div class="hexbox" id="hexContent">...</div>
  </div>
</div>

<script>
let ws, sdrWs, audioCtx=null, audioProcessor=null, sdrQueue=[], sdrPhase=0;
let scanning=false, sdrOn=false, trackerOn=false, diagRunning=false;
const commonFreqs=[315.00,390.00,418.00,430.00,433.05,433.42,433.92,434.42,868.35,915.00];
let rssiHistory=[];
let allSignals=[];

window.onload=()=>{
  const pl=document.getElementById('presetList');
  pl.innerHTML=commonFreqs.map(f=>
    '<button class="btn-preset" onclick="pickPreset('+f+')">'+f.toFixed(2)+'</button>'
  ).join('');
  drawGraph();
};

function drawGraph(){
  const c=document.getElementById('rssiGraph');
  if(!c) return;
  const w=c.width=c.offsetWidth*2, h=c.height=160;
  const ctx=c.getContext('2d');
  ctx.clearRect(0,0,w,h);
  ctx.strokeStyle='#1e2a45'; ctx.lineWidth=2;
  for(let i=0;i<=4;i++){const y=i*h/4; ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke();}
  if(rssiHistory.length<2) return;
  ctx.strokeStyle='#00e5ff'; ctx.lineWidth=3; ctx.beginPath();
  for(let i=0;i<rssiHistory.length;i++){
    const x=i*w/59;
    const y=h-((rssiHistory[i]+110)/70)*h;
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.stroke();
}

function connect(){
  ws=new WebSocket('ws://'+location.host+'/ws');
  ws.onopen=()=>{ send('LIST'); send('GET_SETTINGS'); };
  ws.onclose=()=>{ setSbState('قطع'); setTimeout(connect,2000); };
  ws.onmessage=(e)=>{
    try{ const m=JSON.parse(e.data);
      if(m.type==='SCAN'){
        document.getElementById('anFreq').textContent=m.freq.toFixed(2);
        const pct=Math.max(0,Math.min(100,(m.rssi+110)*1.4));
        const bar=document.getElementById('anBar');bar.style.width=pct+'%';
        bar.style.background=m.rssi>-70?'#00e5ff':m.rssi>-85?'#f9a825':'#ff3d71';
        document.getElementById('anRssi').textContent='RSSI: '+m.rssi+' dBm';
        if(m.noise !== undefined) document.getElementById('anNoise').textContent='Noise: '+m.noise+' dBm';
        setSbFreq(m.freq); setSbRssi(m.rssi);
        rssiHistory.push(m.rssi);
        if(rssiHistory.length>60) rssiHistory.shift();
        drawGraph();
      }
      if(m.type==='SIGNALS'){ allSignals=m.list; renderList(m.list); }
      if(m.type==='STATUS') setSbState(m.msg);
      if(m.type==='SETTINGS') applySettings(m.data);
      if(m.type==='LIVE'){
        document.getElementById('recLiveRssi').textContent=m.rssi+' dBm';
        document.getElementById('recStatus').textContent=m.state;
        setSbRssi(m.rssi);
        if(m.state==='done') setTimeout(()=>document.getElementById('recOverlay').classList.remove('active'),800);
        if(m.state==='timeout') setTimeout(()=>document.getElementById('recOverlay').classList.remove('active'),1500);
      }
      if(m.type==='SDR_FREQ') document.getElementById('sdrFreq').textContent=m.freq.toFixed(2);
      if(m.type==='SDR_STATUS') document.getElementById('sdrStatus').textContent=m.msg;
      if(m.type==='DIAG_OUTPUT'){document.getElementById('diagOutput').innerHTML=m.html;diagRunning=false;}
      if(m.type==='DIAG_SPI_OUTPUT') document.getElementById('spiOutput').innerHTML=m.html;
      if(m.type==='HEX'){
        document.getElementById('hexTitle').textContent='Hex - '+m.name;
        document.getElementById('hexContent').textContent=m.hex;
        document.getElementById('hexModal').classList.add('active');
      }
      if(m.type==='THRESHOLD_SET'){
        document.getElementById('recTh').value=m.value;
        document.getElementById('recThV').textContent=m.value;
      }
      if(m.type==='SENSORS'){
        document.getElementById('tempDisplay').textContent=m.temp.toFixed(1);
        document.getElementById('voltDisplay').textContent=m.volt.toFixed(2);
      }
      if(m.type==='HOPPING'){
        const b=document.getElementById('hopBtn');
        if(b){ b.textContent='📡 پرش فرکانسی: '+(m.on?'روشن':'خاموش'); b.className=m.on?'btn btn-primary':'btn btn-ook'; }
      }
      if(m.type==='TRACKER'){
        const b=document.getElementById('trackerBtn');
        if(b){ b.textContent='🔒 ردیاب: '+(m.on?'روشن':'خاموش'); b.className=m.on?'btn btn-danger':'btn btn-green'; }
      }
      if(m.type==='WEBHOOK'){
        document.getElementById('webhookCheck').checked=m.on;
        if(m.url) document.getElementById('webhookUrl').value=m.url;
      }
    }catch(x){}
  };
}
function send(c){if(ws&&ws.readyState===1)ws.send(c);}

function go(screen){
  document.querySelectorAll('.screen').forEach(s=>s.classList.remove('active'));
  document.getElementById('sc-'+screen).classList.add('active');
  if(screen==='saved') send('LIST');
  if(screen==='sensors') readSensors();
  if(screen!=='analyzer' && scanning){ scanning=false; send('STOP');
    const b=document.getElementById('anBtn'); if(b)b.textContent='▶ شروع اسکن'; }
}

function setSbFreq(f){document.getElementById('sbFreq').textContent=(+f).toFixed(2)+' MHz';}
function setSbRssi(r){document.getElementById('sbRssi').textContent=r+' dBm';}
function setSbState(s){document.getElementById('sbState').textContent='● '+s;}

function pickPreset(f){
  document.getElementById('anTarget').value=f.toFixed(2);
  send('SET_FREQ:'+f); setSbFreq(f);
}
function setBitrate(br){ send('SET_BITRATE:'+br); }
function toggleManchester(){ send('TOGGLE_MANCHESTER'); }
function toggleAdaptiveScan(){
  const enabled=document.getElementById('adaptiveScanCheck').checked;
  send('ADAPTIVE_SCAN:'+enabled);
}
function toggleTracker(){ trackerOn=!trackerOn; send('TOGGLE_TRACKER'); }
function autoThreshold(){ send('AUTO_THRESHOLD'); setSbState('کالیبراسیون...'); }

function beginRecord(){
  const n=(document.getElementById('recName').value||'').trim()||('sig_'+Date.now());
  const cat=(document.getElementById('recCategory').value||'Default').trim();
  const tags=(document.getElementById('recTags').value||'').trim();
  const note=(document.getElementById('recNoteIn').value||'').trim();
  const f=parseFloat(document.getElementById('recFreq').value||'433.92');
  const th=parseInt(document.getElementById('recTh').value);
  if(f<240||f>930){ alert('فرکانس نامعتبر'); return; }
  send('THRESH:'+th);
  send('REC:'+f+':'+n+':'+note+':'+cat+':'+tags);
  document.getElementById('recOverlay').classList.add('active');
  document.getElementById('recLiveRssi').textContent='--- dBm';
  document.getElementById('recStatus').textContent='آماده ضبط — ریموت را فشار بده';
}
function cancelRecord(){ send('STOP'); document.getElementById('recOverlay').classList.remove('active'); }

function toggleScan(){
  scanning=!scanning;
  const b=document.getElementById('anBtn');
  if(scanning){
    const s=parseFloat(document.getElementById('anStart').value);
    const e=parseFloat(document.getElementById('anEnd').value);
    const st=parseFloat(document.getElementById('anStep').value);
    rssiHistory=[];
    send('SCAN_RANGE:'+s+':'+e+':'+st);
    b.textContent='⏹ توقف';
  } else { send('STOP'); b.textContent='▶ شروع اسکن'; }
}

function filterSignals(){
  const q=document.getElementById('searchInput').value.toLowerCase();
  if(!q){ renderList(allSignals); return; }
  const filtered=allSignals.filter(s=>
    s.name.toLowerCase().includes(q) || (s.tags&&s.tags.toLowerCase().includes(q))
  );
  renderList(filtered);
}

function renderList(list){
  const el=document.getElementById('savedList');
  if(!list||!list.length){ el.innerHTML='<div class="empty">خالی</div>'; return; }
  el.innerHTML=list.map(s=>
    '<div class="sig"><div class="sig-head"><div style="flex:1">'+
    '<div class="sig-name">'+s.name+'</div>'+
    '<div class="sig-meta">'+s.freq+' MHz • '+s.len+' bytes</div>'+
    (s.note?'<div class="sig-note">📝 '+s.note+'</div>':'')+
    (s.category?'<div class="sig-category">📂 '+s.category+'</div>':'')+
    (s.tags?'<div class="sig-tags">🏷️ '+s.tags+'</div>':'')+
    '</div></div>'+
    '<div class="sig-actions" style="margin-top:8px">'+
    '<button class="btn btn-green" style="padding:6px 10px;font-size:.75em" onclick="playSig('+s.id+',1,0)">📡 ۱x</button>'+
    '<button class="btn btn-green" style="padding:6px 10px;font-size:.75em" onclick="playSig('+s.id+',3,200)">📡 ۳x</button>'+
    '<button class="btn btn-green" style="padding:6px 10px;font-size:.75em" onclick="playSig('+s.id+',5,300)">📡 ۵x</button>'+
    '<button class="btn btn-ook" style="padding:6px 10px;font-size:.75em" onclick="autoPlay('+s.id+')">🎯</button>'+
    '<button class="btn btn-primary" style="padding:6px 10px;font-size:.75em;flex:0 0 auto;width:auto" onclick="showHex('+s.id+')">🔢</button>'+
    '<button class="btn btn-stop" style="padding:6px 10px;font-size:.75em" onclick="delSig('+s.id+')">🗑</button>'+
    '</div></div>').join('');
}
function playSig(id,count,delay){ send('PLAY_N:'+id+':'+count+':'+delay); }
function autoPlay(id){ send('AUTO_PLAY:'+id); }
function showHex(id){ send('GET_HEX:'+id); }
function closeHex(){ document.getElementById('hexModal').classList.remove('active'); }
function delSig(id){ if(confirm('حذف شود؟')) send('DEL:'+id); }
function clearAll(){ if(confirm('همه حذف شوند؟')) send('CLEAR_ALL'); }

function applySettings(d){
  document.getElementById('recTh').value=d.threshold;
  document.getElementById('recThV').textContent=d.threshold;
  if(d.webhookOn !== undefined) document.getElementById('webhookCheck').checked=d.webhookOn;
  if(d.webhookUrl) document.getElementById('webhookUrl').value=d.webhookUrl;
  if(d.adaptive !== undefined) document.getElementById('adaptiveScanCheck').checked=d.adaptive;
}

function replayAttack(){
  const id=document.getElementById('attackId').value;
  const count=document.getElementById('attackCount').value;
  send('REPLAY_ATTACK:'+id+':'+count);
}
function bruteForce(){
  const f=document.getElementById('bruteFreq').value;
  const count=document.getElementById('bruteCount').value;
  send('BRUTE_FORCE:'+f+':'+count);
}
function toggleHopping(){ send('TOGGLE_HOPPING'); }
function toggleWebhook(){ send('TOGGLE_WEBHOOK'); }
function saveWebhook(){
  const url=document.getElementById('webhookUrl').value;
  send('SAVE_WEBHOOK:'+url);
}
function readSensors(){ send('READ_SENSORS'); }
function startDiagnostic(){ if(!diagRunning){ diagRunning=true; document.getElementById('diagOutput').innerHTML='<div>در حال اجرا...</div>'; send('DIAG_START'); } }
function setSdrFreq(){
  const f=parseFloat(document.getElementById('sdrFreqInput').value);
  if(f<240||f>930){ alert('فرکانس نامعتبر'); return; }
  send('SDR_FREQ:'+f);
  document.getElementById('sdrFreq').textContent=f.toFixed(2);
}
function lockFreq(){
  const f=parseFloat(document.getElementById('anTarget').value);
  if(f<240||f>930){ alert('فرکانس نامعتبر'); return; }
  send('SET_FREQ:'+f);
  document.getElementById('recFreq').value=f.toFixed(2);
  document.getElementById('sdrFreqInput').value=f.toFixed(2);
  setTimeout(()=>{ go('read'); }, 300);
}
function toggleSdr(){
  if(!sdrOn){
    audioCtx=new (window.AudioContext||window.webkitAudioContext)();
    if(audioCtx.state==='suspended') audioCtx.resume();
    sdrQueue=[]; sdrPhase=0;
    const bufSize=1024;
    audioProcessor=audioCtx.createScriptProcessor(bufSize,1,1);
    const ratio=8000/audioCtx.sampleRate;
    audioProcessor.onaudioprocess=(e)=>{
      const out=e.outputBuffer.getChannelData(0);
      for(let i=0;i<out.length;i++){
        if(sdrQueue.length===0){ out[i]=0; continue; }
        out[i]=sdrQueue[0];
        sdrPhase+=ratio;
        while(sdrPhase>=1 && sdrQueue.length>0){ sdrPhase-=1; sdrQueue.shift(); }
      }
    };
    audioProcessor.connect(audioCtx.destination);
    sdrWs=new WebSocket('ws://'+location.host+'/sdr');
    sdrWs.binaryType='arraybuffer';
    sdrWs.onopen=()=>{ send('SDR_START'); };
    sdrWs.onmessage=(e)=>{
      const arr=new Uint8Array(e.data);
      for(let i=0;i<arr.length;i++){ sdrQueue.push((arr[i]-128)/128); }
      while(sdrQueue.length>8192) sdrQueue.shift();
    };
    sdrWs.onclose=()=>{ if(sdrOn) setTimeout(()=>toggleSdr(),500); };
    sdrOn=true;
    document.getElementById('sdrBtn').textContent='⏹ توقف صدا';
    document.getElementById('sdrBtn').className='btn btn-danger';
    document.getElementById('sdrStatus').textContent='در حال پخش...';
  } else {
    sdrOn=false; send('SDR_STOP');
    if(sdrWs){ try{sdrWs.close();}catch(x){} sdrWs=null; }
    if(audioProcessor){ try{audioProcessor.disconnect();}catch(x){} audioProcessor=null; }
    if(audioCtx){ try{audioCtx.close();}catch(x){} audioCtx=null; }
    sdrQueue=[];
    document.getElementById('sdrBtn').textContent='▶ پخش صدا';
    document.getElementById('sdrBtn').className='btn btn-primary';
    document.getElementById('sdrStatus').textContent='متوقف';
  }
}
connect();
</script></body></html>
)HTML";

// ==================== Forward Declarations ====================
byte readRssiReg();
int  readRssi();
byte readRegister(byte reg);
void writeRegister(byte reg, byte val);
void sendStatus(const String&);
void sendList();
void reinitRadio();
void finishRecording();
void runDiagnosticStep();
void performAutoThreshold();
void savePrefs();
void loadPrefs();
void performAdaptiveScan();
void performFrequencyHopping();
void readSensors();
void performTracker();
void doScan();
void performSmartScan();

// ==================== WS Helpers ====================
void sendStatus(const String& m){
  String s=m; s.replace("\\","\\\\"); s.replace("\"","\\\"");
  ws.textAll("{\"type\":\"STATUS\",\"msg\":\""+s+"\"}");
}
void sendList(){
  String out="{\"type\":\"SIGNALS\",\"list\":[";
  for(int i=0;i<signalCount;i++){
    if(i)out+=",";
    String nm=signals[i].name; nm.replace("\"","");
    String nt=""; char np[24]; snprintf(np,sizeof(np),"/n%d.txt",signals[i].id);
    File nf=LittleFS.open(np,"r"); if(nf){ nt=nf.readString(); nf.close(); nt.replace("\"",""); }
    String ct=signals[i].category; ct.replace("\"","");
    String tg=signals[i].tags; tg.replace("\"","");
    out+="{\"id\":"+String(signals[i].id)+",\"name\":\""+nm+"\",";
    out+="\"note\":\""+nt+"\",\"category\":\""+ct+"\",\"tags\":\""+tg+"\",";
    out+="\"freq\":"+String(signals[i].freq,2)+",\"len\":"+String(signals[i].len)+"}";
  }
  out+="]}"; ws.textAll(out);
}
void sendSettings(){
  String out="{\"type\":\"SETTINGS\",\"data\":{";
  out+="\"freq\":"+String(cfgFreq,2);
  out+=",\"bitrate\":"+String(cfgBitrate,2);
  out+=",\"threshold\":"+String(cfgThreshold);
  out+=",\"manchester\":"+String(cfgManchester?"true":"false");
  out+=",\"adaptive\":"+String(adaptiveScanEnabled?"true":"false");
  out+=",\"webhookOn\":"+String(webhookEnabled?"true":"false");
  out+=",\"webhookUrl\":\""+String(webhookUrl)+"\"";
  out+="}}"; ws.textAll(out);
}

// ==================== Direct Register ====================
byte readRegister(byte reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW); delayMicroseconds(3);
  SPI.transfer(reg & 0x7F); delayMicroseconds(3);
  byte result = SPI.transfer(0x00); delayMicroseconds(3);
  digitalWrite(PIN_CS, HIGH); SPI.endTransaction();
  return result;
}
void writeRegister(byte reg, byte val) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW); delayMicroseconds(3);
  SPI.transfer(reg | 0x80); delayMicroseconds(3);
  SPI.transfer(val); delayMicroseconds(3);
  digitalWrite(PIN_CS, HIGH); SPI.endTransaction();
}

// ==================== Sensors ====================
void readSensors() {
  writeRegister(0x0F, 0x00);
  writeRegister(0x12, 0x80);
  byte reg0F = readRegister(0x0F);
  writeRegister(0x0F, reg0F | 0x80);
  delay(5);
  byte adcValue = readRegister(0x11);
  chipTemp = ((float)adcValue) * 0.5f;
  writeRegister(0x0F, 0x08);
  delay(5);
  byte vddValue = readRegister(0x11);
  supplyVoltage = ((float)vddValue) * 0.02f;
  char buf[80];
  snprintf(buf,sizeof(buf),"{\"type\":\"SENSORS\",\"temp\":%.1f,\"volt\":%.2f}", chipTemp, supplyVoltage);
  ws.textAll(buf);
}

// ==================== Adaptive Scan ====================
void performAdaptiveScan() {
  if(noiseSamples < 20) {
    byte raw = readRssiReg();
    int dbm = (int)(0.5f*raw)-131;
    noiseSum += dbm; noiseSamples++;
    if(noiseSamples == 20) {
      noiseFloor = noiseSum / 20;
      adaptiveThreshold = noiseFloor + 10;
      if(adaptiveThreshold > -40) adaptiveThreshold = -40;
    }
  }
  if(currentFreq > scanEnd) currentFreq = scanStart;
  radio.setFrequency(currentFreq);
  radio.startListening();
  delay(30);
  int rssi = readRssi();
  char buf[128];
  snprintf(buf,sizeof(buf),"{\"type\":\"SCAN\",\"freq\":%.2f,\"rssi\":%d,\"noise\":%d}", currentFreq, rssi, noiseFloor);
  ws.textAll(buf);
  currentFreq += scanStep;
}

// ==================== Tracker ====================
void performTracker() {
  if(!trackerEnabled || trackerFreq == 0) return;
  if(millis() - lastTrackerUpdate < TRACKER_INTERVAL) return;
  lastTrackerUpdate = millis();
  radio.setFrequency(trackerFreq);
  radio.startListening();
  delay(30);
  int rssi = readRssi();
  char buf[96];
  snprintf(buf,sizeof(buf),"{\"type\":\"SCAN\",\"freq\":%.2f,\"rssi\":%d}", trackerFreq, rssi);
  ws.textAll(buf);
}

// ==================== Frequency Hopping ====================
void performFrequencyHopping() {
  if (!hoppingEnabled) return;
  if (millis() - lastHopTime < HOP_INTERVAL) return;
  lastHopTime = millis();
  hopIndex = (hopIndex + 1) % 6;
  cfgFreq = hopFreqs[hopIndex];
  radio.setFrequency(cfgFreq);
  radio.startListening();
  char buf[64];
  snprintf(buf,sizeof(buf),"{\"type\":\"SDR_FREQ\",\"freq\":%.2f}", cfgFreq);
  ws.textAll(buf);
}

// ==================== Auto Threshold ====================
void performAutoThreshold() {
  long sum = 0; int n = 0;
  for(int i=0;i<30;i++){
    byte raw = readRssiReg(); int dbm = (int)(0.5f*raw)-131;
    sum += dbm; n++; delay(30);
  }
  int baseline = sum / n; int newTh = baseline + 8;
  if(newTh > -40) newTh = -40;
  if(newTh < -110) newTh = -110;
  cfgThreshold = newTh; savePrefs();
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"type\":\"THRESHOLD_SET\",\"value\":%d}", newTh);
  ws.textAll(buf);
  snprintf(buf, sizeof(buf), "⚙️ baseline: %d → threshold: %d dBm", baseline, newTh);
  sendStatus(String(buf));
}

// ==================== Diagnostic ====================
void sendDiagOutput(String html) {
  html.replace("\"", "\\\""); html.replace("\n", " ");
  ws.textAll("{\"type\":\"DIAG_OUTPUT\",\"html\":\"" + html + "\"}");
}
void sendSpiOutput(String html) {
  html.replace("\"", "\\\""); html.replace("\n", " ");
  ws.textAll("{\"type\":\"DIAG_SPI_OUTPUT\",\"html\":\"" + html + "\"}");
}
void runDiagnosticStep() {
  if (!diagMode) return;
  String out = "";
  if (diagStep == 0) {
    out += "<div>🔍 مرحله ۱: SPI</div>";
    byte ver = readRegister(0x00);
    byte ver2 = readRegister(0x31);
    char buf[64];
    snprintf(buf, sizeof(buf), "<div>0x00: 0x%02X | 0x31: 0x%02X</div>", ver, ver2);
    out += buf;
    if (ver == 0x08 || ver2 == 0x08) out += "<div class='diag-ok'>✅ SPI OK</div>";
    else out += "<div class='diag-fail'>❌ SPI FAIL</div>";
    diagStep++;
  } else if (diagStep == 1) {
    int raw = readRssiReg();
    int dbm = readRssi();
    char buf[64];
    snprintf(buf, sizeof(buf), "<div>RSSI: %d -> %d dBm</div>", raw, dbm);
    out += buf;
    if (raw == 0 || raw == 255) out += "<div class='diag-fail'>❌ نامعتبر</div>";
    else out += "<div class='diag-ok'>✅ معتبر</div>";
    diagStep++;
  } else if (diagStep == 2) {
    readSensors();
    char buf[80];
    snprintf(buf, sizeof(buf), "<div>Temp: %.1f°C | Volt: %.2fV</div>", chipTemp, supplyVoltage);
    out += buf;
    out += "<div class='diag-ok'>✅ سنسورها OK</div>";
    diagStep = 4;
  } else if (diagStep == 4) {
    out += "<div class='diag-ok'>✅ تست کامل شد</div>";
    diagMode = false;
  }
  sendDiagOutput(out);
}

// ==================== File I/O ====================
void saveSigFile(int id,uint8_t* d,uint16_t n){
  char p[24]; snprintf(p,sizeof(p),"/s%d.bin",id);
  File f=LittleFS.open(p,"w"); if(!f)return;
  f.write(d,n); f.close();
}
bool loadSigFile(int id,uint8_t* d,uint16_t* n){
  char p[24]; snprintf(p,sizeof(p),"/s%d.bin",id);
  File f=LittleFS.open(p,"r"); if(!f)return false;
  *n=f.read(d,2048); f.close(); return *n>0;
}
void delSigFile(int id){
  char p[24]; snprintf(p,sizeof(p),"/s%d.bin",id); LittleFS.remove(p);
  char np[24]; snprintf(np,sizeof(np),"/n%d.txt",id); LittleFS.remove(np);
}
void saveNote(int id,const char* note){
  if(!note || !*note) return;
  char p[24]; snprintf(p,sizeof(p),"/n%d.txt",id);
  File f=LittleFS.open(p,"w"); if(!f)return;
  f.print(note); f.close();
}
void loadIndex(){
  signalCount=0;
  File f=LittleFS.open("/idx","r"); if(!f)return;
  f.read((uint8_t*)&signalCount,sizeof(signalCount));
  if(signalCount>MAX_SIGNALS||signalCount<0){signalCount=0;f.close();return;}
  for(int i=0;i<signalCount;i++){
    f.read((uint8_t*)&signals[i],sizeof(Signal));
    if(signals[i].id>=nextId) nextId=signals[i].id+1;
  }
  f.close();
}
void saveIndex(){
  File f=LittleFS.open("/idx","w"); if(!f)return;
  f.write((uint8_t*)&signalCount,sizeof(signalCount));
  for(int i=0;i<signalCount;i++) f.write((uint8_t*)&signals[i],sizeof(Signal));
  f.close();
}

// ==================== Preferences ====================
void loadPrefs(){
  prefs.begin("rfcfg",true);
  cfgFreq=prefs.getFloat("freq",433.92);
  cfgBitrate=prefs.getFloat("bitrate",4.8);
  cfgPower=prefs.getInt("power",20);
  cfgThreshold=prefs.getInt("th",-85);
  cfgOOK=prefs.getBool("ook",true);
  cfgManchester=prefs.getBool("manch",false);
  adaptiveScanEnabled=prefs.getBool("adaptive",true);
  hoppingEnabled=prefs.getBool("hop",false);
  webhookEnabled=prefs.getBool("whOn",false);
  String wh=prefs.getString("whUrl","");
  strncpy(webhookUrl,wh.c_str(),127);
  prefs.end();
}
void savePrefs(){
  prefs.begin("rfcfg",false);
  prefs.putFloat("freq",cfgFreq);
  prefs.putFloat("bitrate",cfgBitrate);
  prefs.putInt("power",cfgPower);
  prefs.putInt("th",cfgThreshold);
  prefs.putBool("ook",cfgOOK);
  prefs.putBool("manch",cfgManchester);
  prefs.putBool("adaptive",adaptiveScanEnabled);
  prefs.putBool("hop",hoppingEnabled);
  prefs.putBool("whOn",webhookEnabled);
  prefs.putString("whUrl",webhookUrl);
  prefs.end();
}

// ==================== Radio Init ====================
void reinitRadio(){
  Serial.print("[Si4432] init... ");
  if(!radio.init()){ Serial.println("FAILED"); sendStatus("خطای Si4432"); return; }
  radio.setFrequency(cfgFreq);
  radio.setBaudRate(cfgBitrate);
  radio.setModulationType(cfgOOK ? Si4432::OOK : Si4432::GFSK);
  radio.setTransmitPower((byte)map(cfgPower,-1,20,0,7));
  radio.setPacketHandling(false);
  radio.setManchesterEncoding(cfgManchester);
  radio.startListening();
  pinMode(PIN_CS,OUTPUT);
  digitalWrite(PIN_CS,HIGH);
  Serial.println("OK");
}

// ==================== RSSI ====================
byte readRssiReg(){ return readRegister(0x26); }
int readRssi(){
  byte maxR=0;
  for(int i=0;i<3;i++){
    byte r=readRssiReg();
    if(r>maxR) maxR=r;
    delayMicroseconds(60);
  }
  return (int)(0.5f*maxR)-131;
}

// ==================== Scan ====================
void doScan(){
  if(!scanning) return;
  if(currentFreq>scanEnd) currentFreq=scanStart;
  radio.setFrequency(currentFreq);
  radio.startListening();
  delay(30);
  int rssi=readRssi();
  ws.textAll("{\"type\":\"SCAN\",\"freq\":"+String(currentFreq,2)+",\"rssi\":"+String(rssi)+"}");
  currentFreq+=scanStep;
}

// ==================== Smart Scan ====================
void performSmartScan() {
  if (!smartScanEnabled) {
    doScan();
    return;
  }

  if (smartScanPhase == 0) {
    if (smartScanFreq > scanEnd) {
      smartScanFreq = scanStart;
      return;
    }
    radio.setFrequency(smartScanFreq);
    radio.startListening();
    delay(30);
    int rssi = readRssi();

    char buf[128];
    snprintf(buf, sizeof(buf), "{\"type\":\"SCAN\",\"freq\":%.2f,\"rssi\":%d}", smartScanFreq, rssi);
    ws.textAll(buf);

    if (rssi > smartScanThreshold) {
      lastStrongFreq = smartScanFreq;
      smartScanPhase = 1;
      float tmp = smartScanFreq - 0.5f;
      smartScanFreq = fmaxf(scanStart, tmp);
      sendStatus("Signal found at " + String(lastStrongFreq) + " MHz");
    } else {
      smartScanFreq += smartScanCoarseStep;
    }
  } else if (smartScanPhase == 1) {
    float tmp = lastStrongFreq + 0.5f;
    float fineEnd = fminf(scanEnd, tmp);
    if (smartScanFreq > fineEnd) {
      smartScanPhase = 0;
      smartScanFreq = lastStrongFreq + smartScanCoarseStep;
      sendStatus("Fine scan complete.");
      return;
    }
    radio.setFrequency(smartScanFreq);
    radio.startListening();
    delay(30);
    int rssi = readRssi();
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"type\":\"SCAN\",\"freq\":%.2f,\"rssi\":%d}", smartScanFreq, rssi);
    ws.textAll(buf);
    smartScanFreq += smartScanFineStep;
  }
}

// ==================== SDR ====================
void doSdr(){
  if(!sdrActive) return;
  unsigned long now=micros();
  if(now-lastSdrSample<SDR_SAMPLE_INTERVAL_US) return;
  lastSdrSample=now;
  byte raw=readRssiReg();
  sdrBuffer[sdrBufIdx++]=raw;
  if(sdrBufIdx>=SDR_BUF_SIZE){
    sdrSocket.binaryAll(sdrBuffer, SDR_BUF_SIZE);
    sdrBufIdx=0;
  }
}

// ==================== Recording ====================
void startRecording(float freq,const char* name,const char* note,const char* cat,const char* tags){
  recState=1;
  recLen=0; recBit=0; recBitCnt=0;
  recStartMs=millis(); recLastMs=millis();
  strncpy(recName,name,NAME_LEN-1); recName[NAME_LEN-1]=0;
  strncpy(recNote,note,63); recNote[63]=0;
  strncpy(recCategory,cat,23); recCategory[23]=0;
  strncpy(recTags,tags,63); recTags[63]=0;
  recFreq=freq;
  radio.setFrequency(recFreq);
  radio.startListening();
  delay(30);
  sendStatus("🎙 آماده ضبط");
}
void finishRecording(){
  recState=0;
  ws.textAll("{\"type\":\"LIVE\",\"rssi\":0,\"state\":\"done\"}");
  if(recLen==0){ sendStatus("❌ سیگنالی دریافت نشد"); return; }
  if(signalCount>=MAX_SIGNALS){ sendStatus("❌ حافظه پر است"); return; }
  int id=nextId++;
  saveSigFile(id,recBuf,recLen);
  saveNote(id, recNote);
  Signal& s=signals[signalCount++];
  s.id=id; s.freq=recFreq; s.len=recLen;
  strncpy(s.name,recName,NAME_LEN-1); s.name[NAME_LEN-1]=0;
  strncpy(s.category,recCategory,23); s.category[23]=0;
  strncpy(s.tags,recTags,63); s.tags[63]=0;
  saveIndex();
  char buf[80];
  snprintf(buf,sizeof(buf),"✅ ذخیره شد: %d bytes",recLen);
  sendStatus(String(buf));
  sendList();
}
void doRecord(){
  if(recState==0) return;
  if(recState==1 && (millis()-recStartMs>REC_START_TIMEOUT_MS)){
    sendStatus("❌ سیگنالی در ۵ ثانیه دریافت نشد");
    recState=0;
    ws.textAll("{\"type\":\"LIVE\",\"rssi\":0,\"state\":\"timeout\"}");
    return;
  }
  int rssi=readRssi();
  bool sig=(rssi>cfgThreshold);
  ws.textAll("{\"type\":\"LIVE\",\"rssi\":"+String(rssi)+
             ",\"state\":\""+String(recState==1?"در انتظار سیگنال":"در حال ضبط")+"\"}");
  if(recState==1){
    if(sig){ recState=2; recLastMs=millis(); }
    delayMicroseconds(150);
    return;
  }
  if(sig){
    recBit |= (0x80>>recBitCnt);
    recLastMs=millis();
  }
  recBitCnt++;
  if(recBitCnt==8){
    if(recLen<2048) recBuf[recLen++]=recBit;
    recBit=0; recBitCnt=0;
  }
  if(millis()-recLastMs>REC_TIMEOUT_MS){
    if(recBitCnt>0 && recLen<2048) recBuf[recLen++]=recBit;
    finishRecording();
  }
  delayMicroseconds(150);
}

// ==================== Replay ====================
void replaySignalN(int id, int count, int delayMs){
  for(int i=0;i<signalCount;i++){
    if(signals[i].id==id){
      static uint8_t buf[2048]; uint16_t len;
      if(!loadSigFile(id,buf,&len)){ sendStatus("❌ فایل یافت نشد"); return; }
      for(int n=0;n<count;n++){
        radio.setFrequency(signals[i].freq);
        radio.turnOn();
        delay(20);
        radio.sendPacket((uint8_t)len,buf);
        delay(delayMs > 0 ? delayMs : 50);
      }
      radio.startListening();
      char b2[80];
      snprintf(b2,sizeof(b2),"📡 ارسال %dx: %s",count,signals[i].name);
      sendStatus(String(b2));
      return;
    }
  }
  sendStatus("❌ یافت نشد");
}
void autoPlaySignal(int id){
  for(int i=0;i<signalCount;i++){
    if(signals[i].id==id){
      static uint8_t buf[2048]; uint16_t len;
      if(!loadSigFile(id,buf,&len)){ sendStatus("❌ فایل یافت نشد"); return; }
      float origBr = cfgBitrate;
      float rates[] = {2.4, 4.8, 9.6};
      for(int r=0;r<3;r++){
        cfgBitrate = rates[r];
        radio.setBaudRate(cfgBitrate);
        radio.setFrequency(signals[i].freq);
        radio.turnOn();
        delay(30);
        radio.sendPacket((uint8_t)len,buf);
        delay(80);
        char b2[80];
        snprintf(b2,sizeof(b2),"🎯 ارسال %.1f kbps",rates[r]);
        sendStatus(String(b2));
        delay(150);
      }
      cfgBitrate = origBr;
      radio.setBaudRate(cfgBitrate);
      radio.startListening();
      sendStatus("✅ Auto-play کامل شد");
      return;
    }
  }
}
void showHexSignal(int id){
  for(int i=0;i<signalCount;i++){
    if(signals[i].id==id){
      static uint8_t buf[2048]; uint16_t len;
      if(!loadSigFile(id,buf,&len)){ sendStatus("❌ فایل یافت نشد"); return; }
      String hex = "";
      for(uint16_t i=0;i<len;i++){
        char b[4]; snprintf(b,sizeof(b),"%02X ",buf[i]);
        hex += b;
        if((i+1)%16 == 0) hex += "\n";
      }
      String nm = signals[i].name; nm.replace("\"","");
      ws.textAll("{\"type\":\"HEX\",\"name\":\""+nm+"\",\"hex\":\""+hex+"\"}");
      return;
    }
  }
}
void deleteSignal(int id){
  for(int i=0;i<signalCount;i++){
    if(signals[i].id==id){
      delSigFile(id);
      for(int j=i;j<signalCount-1;j++) signals[j]=signals[j+1];
      signalCount--;
      saveIndex();
      sendList();
      sendStatus("🗑 حذف شد");
      return;
    }
  }
}
void clearAllSignals(){
  for(int i=0;i<signalCount;i++) delSigFile(signals[i].id);
  signalCount=0; saveIndex();
  sendList();
  sendStatus("🗑 همه حذف شدند");
}

// ==================== WebSocket ====================
void onWsEvent(AsyncWebSocket* s,AsyncWebSocketClient* c,AwsEventType t,void* a,uint8_t* d,size_t l){
  if(t!=WS_EVT_DATA)return;
  String cmd; cmd.reserve(l+1);
  for(size_t i=0;i<l;i++) cmd+=(char)d[i];
  cmd.trim();

  if(cmd=="SCAN"){ scanning=true; sdrActive=false; recState=0;
    scanStart=300; scanEnd=928; scanStep=0.5; currentFreq=scanStart;
    smartScanFreq=scanStart; smartScanPhase=0;
    sendStatus("در حال اسکن..."); }
  else if(cmd.startsWith("SCAN_RANGE:")){
    int p1=cmd.indexOf(':',11); int p2=cmd.indexOf(':',p1+1);
    if(p1>0 && p2>0){
      scanStart=cmd.substring(11,p1).toFloat();
      scanEnd=cmd.substring(p1+1,p2).toFloat();
      scanStep=cmd.substring(p2+1).toFloat();
      if(scanStep<0.05) scanStep=0.05;
      currentFreq=scanStart;
      smartScanFreq=scanStart;
      scanning=true; sdrActive=false; recState=0;
      smartScanPhase=0;
      sendStatus("اسکن...");
    }
  }
  else if(cmd.startsWith("ADAPTIVE_SCAN:")){
    adaptiveScanEnabled=(cmd.substring(14)=="true");
    savePrefs();
    sendStatus(adaptiveScanEnabled?"اسکن تطبیقی روشن":"اسکن تطبیقی خاموش");
  }
  else if(cmd=="STOP"){ scanning=false; if(recState)finishRecording(); sendStatus("متوقف"); }
  else if(cmd=="LIST"){ sendList(); }
  else if(cmd=="GET_SETTINGS"){ sendSettings(); }
  else if(cmd=="CLEAR_ALL"){ clearAllSignals(); }
  else if(cmd=="AUTO_THRESHOLD"){ performAutoThreshold(); }
  else if(cmd=="READ_SENSORS"){ readSensors(); }
  else if(cmd=="TOGGLE_HOPPING"){
    hoppingEnabled=!hoppingEnabled; savePrefs();
    char buf[64]; snprintf(buf,sizeof(buf),"{\"type\":\"HOPPING\",\"on\":%s}", hoppingEnabled?"true":"false");
    ws.textAll(buf);
    sendStatus(hoppingEnabled?"پرش فرکانسی روشن":"پرش فرکانسی خاموش");
  }
  else if(cmd=="TOGGLE_TRACKER"){
    trackerEnabled=!trackerEnabled;
    if(trackerEnabled) trackerFreq=currentFreq;
    char buf[64]; snprintf(buf,sizeof(buf),"{\"type\":\"TRACKER\",\"on\":%s}", trackerEnabled?"true":"false");
    ws.textAll(buf);
    sendStatus(trackerEnabled?"ردیاب روشن":"ردیاب خاموش");
  }
  else if(cmd=="TOGGLE_MANCHESTER"){
    cfgManchester=!cfgManchester; savePrefs(); reinitRadio();
    sendSettings();
    sendStatus(cfgManchester?"Manchester روشن":"Manchester خاموش");
  }
  else if(cmd.startsWith("REC:")){
    int p1=cmd.indexOf(':',4);
    int p2=cmd.indexOf(':',p1+1);
    int p3=cmd.indexOf(':',p2+1);
    int p4=cmd.indexOf(':',p3+1);
    if(p1>0 && p2>0 && p3>0 && p4>0){
      float f=cmd.substring(4,p1).toFloat();
      String n=cmd.substring(p1+1,p2);
      String nt=cmd.substring(p2+1,p3);
      String ct=cmd.substring(p3+1,p4);
      String tg=cmd.substring(p4+1);
      if(f<240||f>930){ sendStatus("❌ فرکانس نامعتبر"); return; }
      scanning=false; sdrActive=false;
      startRecording(f,n.c_str(),nt.c_str(),ct.c_str(),tg.c_str());
    }
  }
  else if(cmd.startsWith("PLAY_N:")){
    int p1=cmd.indexOf(':',7); int p2=cmd.indexOf(':',p1+1);
    if(p1>0 && p2>0){
      int id=cmd.substring(7,p1).toInt();
      int cnt=cmd.substring(p1+1,p2).toInt();
      int dl=cmd.substring(p2+1).toInt();
      replaySignalN(id, cnt, dl);
    }
  }
  else if(cmd.startsWith("AUTO_PLAY:")){ int id=cmd.substring(10).toInt(); autoPlaySignal(id); }
  else if(cmd.startsWith("GET_HEX:")){ int id=cmd.substring(8).toInt(); showHexSignal(id); }
  else if(cmd.startsWith("DEL:")){ deleteSignal(cmd.substring(4).toInt()); }
  else if(cmd.startsWith("REPLAY_ATTACK:")){
    int p1=cmd.indexOf(':',14);
    if(p1>0){ int id=cmd.substring(14,p1).toInt(); int cnt=cmd.substring(p1+1).toInt(); replaySignalN(id, cnt, 200); }
  }
  else if(cmd.startsWith("BRUTE_FORCE:")){
    int p1=cmd.indexOf(':',12);
    if(p1>0){ int cnt=cmd.substring(p1+1).toInt(); sendStatus("💥 Brute Force شروع شد"); }
  }
  else if(cmd.startsWith("SET_BITRATE:")){
    float br=cmd.substring(12).toFloat();
    if(br>=0.5&&br<=128){
      cfgBitrate=br; radio.setBaudRate(br); savePrefs();
      ws.textAll("{\"type\":\"BITRATE\",\"value\":"+String(br,1)+"}");
    }
  }
  else if(cmd.startsWith("THRESH:")){ cfgThreshold=cmd.substring(7).toInt(); savePrefs(); }
  else if(cmd.startsWith("SET_FREQ:")){
    float f=cmd.substring(9).toFloat();
    if(f>=240&&f<=930){ cfgFreq=f; radio.setFrequency(f); savePrefs(); }
  }
  else if(cmd=="SDR_START"){
    scanning=false; recState=0;
    sdrActive=true; sdrBufIdx=0; lastSdrSample=micros();
    radio.setFrequency(cfgFreq); radio.startListening(); delay(30);
    ws.textAll("{\"type\":\"SDR_STATUS\",\"msg\":\"در حال پخش صدا\"}");
    ws.textAll("{\"type\":\"SDR_FREQ\",\"freq\":"+String(cfgFreq,2)+"}");
  }
  else if(cmd=="SDR_STOP"){ sdrActive=false; ws.textAll("{\"type\":\"SDR_STATUS\",\"msg\":\"متوقف\"}"); }
  else if(cmd.startsWith("SDR_FREQ:")){
    float f=cmd.substring(9).toFloat();
    if(f>=240&&f<=930){
      cfgFreq=f; radio.setFrequency(f); savePrefs();
      ws.textAll("{\"type\":\"SDR_FREQ\",\"freq\":"+String(f,2)+"}");
    }
  }
  else if(cmd=="SDR_SCAN"){
    sdrActive=false;
    float bestF=433.92; int bestR=-200;
    float testF[]={144.00,145.00,433.05,433.42,433.92,434.42,446.00,868.10,868.35,915.00};
    for(int i=0;i<10;i++){
      radio.setFrequency(testF[i]); radio.startListening(); delay(40);
      int r=readRssi();
      if(r>bestR){ bestR=r; bestF=testF[i]; }
    }
    cfgFreq=bestF; radio.setFrequency(bestF); savePrefs();
    ws.textAll("{\"type\":\"SDR_FREQ\",\"freq\":"+String(bestF,2)+"}");
    char buf[100];
    snprintf(buf,sizeof(buf),"{\"type\":\"SDR_STATUS\",\"msg\":\"بهترین: %.2f MHz (%d dBm)\"}",bestF,bestR);
    ws.textAll(buf);
  }
  else if(cmd=="DIAG_START"){ diagMode=true; diagStep=0; diagComplete=false; runDiagnosticStep(); }
  else if(cmd=="DIAG_SPI"){
    byte v1=readRegister(0x00); byte v2=readRegister(0x31);
    char buf[160];
    snprintf(buf,sizeof(buf),"<div>0x00 = 0x%02X</div><div>0x31 = 0x%02X</div>",v1,v2);
    String s=String(buf);
    if(v1==0x08||v2==0x08) s+="<div class='diag-ok'>✅ SPI OK</div>";
    else s+="<div class='diag-fail'>❌ SPI FAIL</div>";
    sendSpiOutput(s);
  }
  else if(cmd=="TOGGLE_WEBHOOK"){
    webhookEnabled=!webhookEnabled; savePrefs();
    char buf[64];
    snprintf(buf,sizeof(buf),"{\"type\":\"WEBHOOK\",\"on\":%s}", webhookEnabled?"true":"false");
    ws.textAll(buf);
  }
  else if(cmd.startsWith("SAVE_WEBHOOK:")){
    String url=cmd.substring(13);
    strncpy(webhookUrl,url.c_str(),127);
    savePrefs();
    sendStatus("✅ Webhook ذخیره شد");
  }
}

void onSdrWsEvent(AsyncWebSocket* s,AsyncWebSocketClient* c,AwsEventType t,void* a,uint8_t* d,size_t l){}

// ==================== Setup / Loop ====================
void setup(){
  Serial.begin(115200); delay(500);
  Serial.println("\n=== Si4432 Ultimate ===");
  if(!LittleFS.begin(true)) Serial.println("FS failed");
  loadIndex();
  loadPrefs();
  reinitRadio();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID,AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  sdrSocket.onEvent(onSdrWsEvent);
  server.addHandler(&sdrSocket);

  server.on("/",HTTP_GET,[](AsyncWebServerRequest* r){
    r->send_P(200,"text/html; charset=utf-8",HTML);
  });
  ElegantOTA.begin(&server);
  server.begin();
  Serial.println("Ready — connect to WiFi 'RF-Recorder' pass '12345678'");
}

void loop(){
  ElegantOTA.loop();
  ws.cleanupClients();
  sdrSocket.cleanupClients();

  if(sdrActive){
    doSdr();
  } else {
    if(hoppingEnabled) performFrequencyHopping();
    if(trackerEnabled) performTracker();
    if(scanning){
      if(adaptiveScanEnabled) performAdaptiveScan();
      else doScan();
    }
    if(recState) doRecord();
    delay(1);
  }
}
