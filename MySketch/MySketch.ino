/*
 * Si4432 Pro - RF Remote Tool with Analyzer + SDR
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

// ==================== State ====================
bool scanning = false;
bool sdrActive = false;
int  recState = 0;
float currentFreq = 433.92;
float scanStart = 300.0;
float scanEnd   = 928.0;
float scanStep  = 0.5;

float recFreq = 0;
uint8_t  recBuf[2048];
uint16_t recLen = 0;
uint8_t  recBit = 0;
uint8_t  recBitCnt = 0;
unsigned long recStartMs = 0;
unsigned long recLastMs = 0;
char     recName[32] = "";

#define REC_TIMEOUT_MS 300
#define REC_START_TIMEOUT_MS 5000

// ==================== SDR ====================
#define SDR_BUF_SIZE 128
uint8_t sdrBuffer[SDR_BUF_SIZE];
int sdrBufIdx = 0;
unsigned long lastSdrSample = 0;
#define SDR_SAMPLE_INTERVAL_US 125

// ==================== Signals ====================
#define MAX_SIGNALS 32
#define NAME_LEN    32
struct Signal { uint16_t id; float freq; uint16_t len; char name[NAME_LEN]; };
Signal signals[MAX_SIGNALS];
int signalCount = 0;
int nextId = 1;

// ==================== Web ====================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocket sdrSocket("/sdr");

// ==================== HTML ====================
const char HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="fa" dir="rtl"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Si4432 Pro</title><style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;user-select:none}
body{font-family:Tahoma,sans-serif;background:#0a0e17;color:#e0e6ed;
     max-width:520px;margin:auto;min-height:100vh;display:flex;flex-direction:column}
#statusbar{background:#0d1420;padding:8px 12px;display:flex;justify-content:space-between;
  align-items:center;border-bottom:1px solid #1e2a45;font-family:monospace;font-size:.8em;
  min-height:40px;position:sticky;top:0;z-index:10}
#sbFreq{color:#00e5ff;font-weight:bold}
#sbRssi{color:#f9a825}
#sbState{color:#8899aa}
.screen{display:none;padding:12px;flex:1;overflow-y:auto}
.screen.active{display:block}
.header{display:flex;align-items:center;gap:10px;margin-bottom:12px;
  padding-bottom:8px;border-bottom:1px solid #1e2a45}
.back{background:transparent;color:#00e5ff;border:none;font-size:1.4em;
  cursor:pointer;padding:4px 12px;flex:0 0 auto}
.title{font-size:1.05em;font-weight:bold;color:#e0e6ed;flex:1}
.menu{display:grid;grid-template-columns:1fr 1fr;gap:10px;padding:8px 0}
.menu-item{background:#151c2c;border:1px solid #1e2a45;border-radius:12px;
  padding:22px 10px;display:flex;flex-direction:column;align-items:center;
  justify-content:center;gap:10px;cursor:pointer;transition:all .15s;min-height:120px}
.menu-item:active{transform:scale(.96);background:#1e2a45}
.menu-icon{font-size:2.4em;line-height:1}
.menu-label{font-size:.85em;color:#e0e6ed;text-align:center;font-weight:600}
.menu-sub{font-size:.7em;color:#7a8ba8;text-align:center;margin-top:2px}
.card{background:#151c2c;border-radius:10px;padding:12px;margin-bottom:10px;border:1px solid #1e2a45}
.row{display:flex;gap:6px;margin-bottom:8px;align-items:center;flex-wrap:wrap}
.label{color:#7a8ba8;font-size:.8em;min-width:65px}
input[type=number],input[type=text]{flex:1;background:#0a0e17;color:#0f0;
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
.btn-preset{background:#34495e;color:#fff;padding:8px 4px;font-size:.7em;flex:1;min-width:60px;margin:2px}
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
.sig-actions{display:flex;gap:4px;flex:0 0 auto}
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
</style></head><body>

<div id="statusbar">
  <span id="sbFreq">433.92 MHz</span>
  <span id="sbRssi">-- dBm</span>
  <span id="sbState">● idle</span>
</div>

<div id="sc-home" class="screen active">
  <div class="header"><div class="title">📡 Si4432 Pro</div></div>
  <div class="menu">
    <div class="menu-item" onclick="go('read')">
      <span class="menu-icon">📻</span>
      <span class="menu-label">Read</span>
      <span class="menu-sub">ضبط سیگنال</span>
    </div>
    <div class="menu-item" onclick="go('saved')">
      <span class="menu-icon">📼</span>
      <span class="menu-label">Saved</span>
      <span class="menu-sub">سیگنال‌ها</span>
    </div>
    <div class="menu-item" onclick="go('analyzer')">
      <span class="menu-icon">🔍</span>
      <span class="menu-label">Analyzer</span>
      <span class="menu-sub">یافتن فرکانس</span>
    </div>
    <div class="menu-item" onclick="go('sdr')">
      <span class="menu-icon">🔊</span>
      <span class="menu-label">SDR</span>
      <span class="menu-sub">شنیدن صدا</span>
    </div>
  </div>
</div>

<div id="sc-read" class="screen">
  <div class="header">
    <button class="back" onclick="go('home')">←</button>
    <div class="title">📻 Read</div>
  </div>
  <div class="card">
    <div class="row"><span class="label">اسم</span>
      <input type="text" id="recName" placeholder="درب پارکینگ" maxlength="30"></div>
    <div class="row"><span class="label">فرکانس</span>
      <input type="number" id="recFreq" step="0.01" value="433.92" min="240" max="930"></div>
    <div class="row"><span class="label">بیت‌ریت</span>
      <button class="btn-preset" id="rbr24" onclick="setBitrate(2.4)">2.4</button>
      <button class="btn-preset active" id="rbr48" onclick="setBitrate(4.8)">4.8</button>
      <button class="btn-preset" id="rbr96" onclick="setBitrate(9.6)">9.6</button>
    </div>
    <div class="row"><span class="label">آستانه</span>
      <input type="range" id="recTh" min="-110" max="-40" value="-85"
             oninput="document.getElementById('recThV').textContent=this.value">
      <span class="th-val" id="recThV">-85</span>
    </div>
    <button class="btn btn-primary" style="margin-top:10px" onclick="beginRecord()">🔴 شروع ضبط</button>
    <div class="hint">ریموت را ۲-۳ سانتی‌متر از آنتن فشار بده</div>
  </div>
</div>

<div id="sc-saved" class="screen">
  <div class="header">
    <button class="back" onclick="go('home')">←</button>
    <div class="title">📼 Saved</div>
    <button class="btn btn-stop" style="flex:0 0 auto;padding:6px 10px;font-size:.8em"
            onclick="clearAll()">🗑 همه</button>
  </div>
  <div id="savedList"><div class="empty">خالی</div></div>
</div>

<div id="sc-analyzer" class="screen">
  <div class="header">
    <button class="back" onclick="go('home')">←</button>
    <div class="title">🔍 Analyzer</div>
  </div>
  <div class="display">
    <span class="value" id="anFreq">---.--</span>
    <div class="unit">MHz</div>
    <div class="rssi-bar"><div class="rssi-fill" id="anBar"></div></div>
    <div class="hint" id="anRssi">RSSI: --- dBm</div>
  </div>

  <div class="card">
    <div style="color:#00e5ff;font-size:.9em;margin-bottom:8px">📡 فرکانس‌های رایج ریموت</div>
    <div class="presets" id="presetList"></div>
    <div class="hint" style="margin-top:8px">یک فرکانس انتخاب کن و ریموت را فشار بده</div>
  </div>

  <div class="card">
    <div style="color:#00e5ff;font-size:.9em;margin-bottom:8px">🔎 اسکن کامل</div>
    <div class="row"><span class="label">شروع</span>
      <input type="number" id="anStart" step="0.5" value="300.0"></div>
    <div class="row"><span class="label">پایان</span>
      <input type="number" id="anEnd" step="0.5" value="928.0"></div>
    <div class="row"><span class="label">گام</span>
      <input type="number" id="anStep" step="0.05" value="0.5"></div>
    <div class="btn-row">
      <button class="btn btn-primary" id="anBtn" onclick="toggleScan()">▶ شروع اسکن</button>
    </div>
  </div>

  <div class="card">
    <div class="row"><span class="label">قفل روی</span>
      <input type="number" id="anTarget" step="0.01" value="433.92"></div>
    <button class="btn btn-green" style="width:100%;margin-top:6px"
            onclick="lockFreq()">🔒 قفل و رفتن به ضبط</button>
    <div class="hint">فرکانس قفل‌شده در صفحه Read قرار می‌گیرد</div>
  </div>
</div>

<div id="sc-sdr" class="screen">
  <div class="header">
    <button class="back" onclick="go('home')">←</button>
    <div class="title">🔊 SDR</div>
  </div>

  <div class="display">
    <span class="value" id="sdrFreq">---.--</span>
    <div class="unit">MHz</div>
    <div class="hint" id="sdrStatus">متوقف</div>
  </div>

  <div class="card">
    <div class="row"><span class="label">فرکانس</span>
      <input type="number" id="sdrFreqInput" step="0.01" value="433.92" min="240" max="930">
      <button class="btn btn-green" style="flex:0 0 auto;padding:10px 14px"
              onclick="setSdrFreq()">تنظیم</button></div>
    <button class="btn btn-primary" id="sdrBtn" onclick="toggleSdr()" style="margin-top:8px">▶ پخش صدا</button>
    <div class="hint" style="margin-top:8px">روی فرکانس مکالمه یا صدا تنظیم کن و پخش را بزن</div>
  </div>

  <div class="card">
    <div class="row"><span class="label">اسکن صوتی</span></div>
    <button class="btn btn-ook" style="width:100%;margin-top:6px"
            onclick="send('SDR_SCAN')">🔍 پیدا کردن سیگنال صوتی</button>
    <div class="hint">اتوماتیک فرکانس‌ها را چک می‌کند و روی قوی‌ترین توقف می‌کند</div>
  </div>

  <div class="card">
    <div style="color:#00e5ff;font-size:.9em;margin-bottom:8px">📻 فرکانس‌های رایج صوتی</div>
    <div class="presets">
      <button class="btn-preset" onclick="quickSdr(144.00)">144.00</button>
      <button class="btn-preset" onclick="quickSdr(145.00)">145.00</button>
      <button class="btn-preset" onclick="quickSdr(433.50)">433.50</button>
      <button class="btn-preset" onclick="quickSdr(446.00)">446.00</button>
      <button class="btn-preset" onclick="quickSdr(868.10)">868.10</button>
      <button class="btn-preset" onclick="quickSdr(915.00)">915.00</button>
    </div>
  </div>
</div>

<div class="rec-overlay" id="recOverlay">
  <div class="rec-pulse"></div>
  <div class="rec-rssi" id="recLiveRssi">--- dBm</div>
  <div class="rec-status" id="recStatus">آماده ضبط — ریموت را فشار بده</div>
  <button class="rec-cancel" onclick="cancelRecord()">لغو</button>
</div>

<script>
let ws, sdrWs, audioCtx=null, audioProcessor=null, sdrQueue=[], sdrPhase=0;
let scanning=false, sdrOn=false;
const commonFreqs=[315.00,390.00,418.00,430.00,433.05,433.42,433.92,434.42,868.35,915.00];

window.onload=()=>{
  const pl=document.getElementById('presetList');
  pl.innerHTML=commonFreqs.map(f=>
    '<button class="btn-preset" onclick="pickPreset('+f+')">'+f.toFixed(2)+'</button>'
  ).join('');
};

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
        setSbFreq(m.freq); setSbRssi(m.rssi);
      }
      if(m.type==='SIGNALS') renderList(m.list);
      if(m.type==='STATUS') setSbState(m.msg);
      if(m.type==='SETTINGS') applySettings(m.data);
      if(m.type==='BITRATE') setBitrateUI(m.value);
      if(m.type==='LIVE'){
        document.getElementById('recLiveRssi').textContent=m.rssi+' dBm';
        document.getElementById('recStatus').textContent=m.state;
        setSbRssi(m.rssi);
        if(m.state==='done') setTimeout(()=>document.getElementById('recOverlay').classList.remove('active'),800);
        if(m.state==='timeout') setTimeout(()=>document.getElementById('recOverlay').classList.remove('active'),1500);
      }
      if(m.type==='SDR_FREQ'){document.getElementById('sdrFreq').textContent=m.freq.toFixed(2);}
      if(m.type==='SDR_STATUS'){document.getElementById('sdrStatus').textContent=m.msg;}
    }catch(x){}
  };
}
function send(c){if(ws&&ws.readyState===1)ws.send(c);}

function go(screen){
  document.querySelectorAll('.screen').forEach(s=>s.classList.remove('active'));
  document.getElementById('sc-'+screen).classList.add('active');
  if(screen==='saved') send('LIST');
  if(screen!=='analyzer' && scanning){ scanning=false; send('STOP');
    const b=document.getElementById('anBtn'); if(b)b.textContent='▶ شروع اسکن'; }
}

function setSbFreq(f){document.getElementById('sbFreq').textContent=(+f).toFixed(2)+' MHz';}
function setSbRssi(r){document.getElementById('sbRssi').textContent=r+' dBm';}
function setSbState(s){document.getElementById('sbState').textContent='● '+s;}

function pickPreset(f){
  document.getElementById('anTarget').value=f.toFixed(2);
  send('SET_FREQ:'+f);
  setSbFreq(f);
}

function setBitrate(br){ send('SET_BITRATE:'+br); }
function setBitrateUI(br){
  ['rbr24','rbr48','rbr96'].forEach(id=>{
    const el=document.getElementById(id); if(el) el.classList.remove('active');
  });
  if(br==2.4){const e=document.getElementById('rbr24'); if(e)e.classList.add('active');}
  else if(br==4.8){const e=document.getElementById('rbr48'); if(e)e.classList.add('active');}
  else if(br==9.6){const e=document.getElementById('rbr96'); if(e)e.classList.add('active');}
}

function beginRecord(){
  const n=(document.getElementById('recName').value||'').trim()||('sig_'+Date.now());
  const f=parseFloat(document.getElementById('recFreq').value||'433.92');
  const th=parseInt(document.getElementById('recTh').value);
  if(f<240||f>930){ alert('فرکانس نامعتبر'); return; }
  send('THRESH:'+th);
  send('REC:'+f+':'+n);
  document.getElementById('recOverlay').classList.add('active');
  document.getElementById('recLiveRssi').textContent='--- dBm';
  document.getElementById('recStatus').textContent='آماده ضبط — ریموت را فشار بده';
}
function cancelRecord(){
  send('STOP');
  document.getElementById('recOverlay').classList.remove('active');
}

function toggleScan(){
  scanning=!scanning;
  const b=document.getElementById('anBtn');
  if(scanning){
    const s=parseFloat(document.getElementById('anStart').value);
    const e=parseFloat(document.getElementById('anEnd').value);
    const st=parseFloat(document.getElementById('anStep').value);
    send('SCAN_RANGE:'+s+':'+e+':'+st);
    b.textContent='⏹ توقف';
  } else { send('STOP'); b.textContent='▶ شروع اسکن'; }
}
function lockFreq(){
  const f=parseFloat(document.getElementById('anTarget').value);
  if(f<240||f>930){ alert('فرکانس نامعتبر'); return; }
  send('SET_FREQ:'+f);
  document.getElementById('recFreq').value=f.toFixed(2);
  document.getElementById('sdrFreqInput').value=f.toFixed(2);
  setTimeout(()=>{ go('read'); }, 300);
}

function renderList(list){
  const el=document.getElementById('savedList');
  if(!list||!list.length){ el.innerHTML='<div class="empty">خالی</div>'; return; }
  el.innerHTML=list.map(s=>
    '<div class="sig"><div class="sig-head"><div style="flex:1">'+
    '<div class="sig-name">'+s.name+'</div>'+
    '<div class="sig-meta">'+s.freq+' MHz • '+s.len+' bytes</div></div>'+
    '<div class="sig-actions">'+
    '<button class="btn btn-green" style="padding:8px 12px" onclick="playSig('+s.id+')">📡</button>'+
    '<button class="btn btn-stop" style="padding:8px 12px" onclick="delSig('+s.id+')">🗑</button>'+
    '</div></div></div>').join('');
}
function playSig(id){ send('PLAY:'+id); }
function delSig(id){ if(confirm('حذف شود؟')) send('DEL:'+id); }
function clearAll(){ if(confirm('همه حذف شوند؟')) send('CLEAR_ALL'); }

function applySettings(d){
  document.getElementById('recTh').value=d.threshold;
  document.getElementById('recThV').textContent=d.threshold;
  setBitrateUI(d.bitrate);
  setSbFreq(d.freq);
}

function toggleSdr(){
  if(!sdrOn){
    audioCtx=new (window.AudioContext||window.webkitAudioContext)();
    if(audioCtx.state==='suspended') audioCtx.resume();
    sdrQueue=[];
    sdrPhase=0;

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
      for(let i=0;i<arr.length;i++){
        const v=(arr[i]-128)/128;
        sdrQueue.push(v);
      }
      while(sdrQueue.length>8192) sdrQueue.shift();
    };
    sdrWs.onclose=()=>{ if(sdrOn) setTimeout(()=>toggleSdr(),500); };

    sdrOn=true;
    document.getElementById('sdrBtn').textContent='⏹ توقف صدا';
    document.getElementById('sdrBtn').className='btn btn-danger';
    document.getElementById('sdrStatus').textContent='در حال پخش...';
  } else {
    sdrOn=false;
    send('SDR_STOP');
    if(sdrWs){ try{sdrWs.close();}catch(x){} sdrWs=null; }
    if(audioProcessor){ try{audioProcessor.disconnect();}catch(x){} audioProcessor=null; }
    if(audioCtx){ try{audioCtx.close();}catch(x){} audioCtx=null; }
    sdrQueue=[];
    document.getElementById('sdrBtn').textContent='▶ پخش صدا';
    document.getElementById('sdrBtn').className='btn btn-primary';
    document.getElementById('sdrStatus').textContent='متوقف';
  }
}
function setSdrFreq(){
  const f=parseFloat(document.getElementById('sdrFreqInput').value);
  if(f<240||f>930){ alert('فرکانس نامعتبر'); return; }
  send('SDR_FREQ:'+f);
  document.getElementById('sdrFreq').textContent=f.toFixed(2);
}
function quickSdr(f){
  document.getElementById('sdrFreqInput').value=f.toFixed(2);
  setSdrFreq();
}

connect();
</script></body></html>
)HTML";

// ==================== Forward ====================
byte readRssiReg();
int  readRssi();
void sendStatus(const String&);
void sendList();
void reinitRadio();
void finishRecording();

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
    out+="{\"id\":"+String(signals[i].id)+",\"name\":\""+nm+"\",";
    out+="\"freq\":"+String(signals[i].freq,2)+",\"len\":"+String(signals[i].len)+"}";
  }
  out+="]}";
  ws.textAll(out);
}
void sendSettings(){
  String out="{\"type\":\"SETTINGS\",\"data\":{";
  out+="\"freq\":"+String(cfgFreq,2);
  out+=",\"bitrate\":"+String(cfgBitrate,2);
  out+=",\"threshold\":"+String(cfgThreshold);
  out+="}}"; ws.textAll(out);
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
  char p[24]; snprintf(p,sizeof(p),"/s%d.bin",id);
  LittleFS.remove(p);
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
  prefs.end();
}
void savePrefs(){
  prefs.begin("rfcfg",false);
  prefs.putFloat("freq",cfgFreq);
  prefs.putFloat("bitrate",cfgBitrate);
  prefs.putInt("power",cfgPower);
  prefs.putInt("th",cfgThreshold);
  prefs.putBool("ook",cfgOOK);
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
  radio.setManchesterEncoding(false);
  radio.startListening();
  pinMode(PIN_CS,OUTPUT);
  digitalWrite(PIN_CS,HIGH);
  Serial.println("OK");
}

// ==================== RSSI Direct SPI ====================
byte readRssiReg(){
  SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));
  digitalWrite(PIN_CS,LOW);
  delayMicroseconds(3);
  SPI.transfer(0x26);
  delayMicroseconds(3);
  byte r=SPI.transfer(0x00);
  delayMicroseconds(3);
  digitalWrite(PIN_CS,HIGH);
  SPI.endTransaction();
  return r;
}
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
void startRecording(float freq,const char* name){
  recState=1;
  recLen=0; recBit=0; recBitCnt=0;
  recStartMs=millis(); recLastMs=millis();
  strncpy(recName,name,NAME_LEN-1); recName[NAME_LEN-1]=0;
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
  Signal& s=signals[signalCount++];
  s.id=id; s.freq=recFreq; s.len=recLen;
  strncpy(s.name,recName,NAME_LEN-1); s.name[NAME_LEN-1]=0;
  saveIndex();
  sendStatus("✅ ذخیره شد: "+String(recLen)+" bytes");
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
void replaySignal(int id){
  for(int i=0;i<signalCount;i++){
    if(signals[i].id==id){
      static uint8_t buf[2048]; uint16_t len;
      if(!loadSigFile(id,buf,&len)){ sendStatus("❌ فایل یافت نشد"); return; }
      radio.setFrequency(signals[i].freq);
      radio.turnOn();
      delay(20);
      radio.sendPacket((uint8_t)len,buf);
      delay(60);
      radio.startListening();
      sendStatus("📡 ارسال شد: "+String(signals[i].name));
      return;
    }
  }
  sendStatus("❌ یافت نشد");
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

// ==================== WebSocket Control ====================
void onWsEvent(AsyncWebSocket* s,AsyncWebSocketClient* c,AwsEventType t,void* a,uint8_t* d,size_t l){
  if(t!=WS_EVT_DATA)return;
  String cmd; cmd.reserve(l+1);
  for(size_t i=0;i<l;i++) cmd+=(char)d[i];
  cmd.trim();

  if(cmd=="SCAN"){ scanning=true; sdrActive=false; recState=0;
    scanStart=300; scanEnd=928; scanStep=0.5; currentFreq=scanStart;
    sendStatus("در حال اسکن..."); }
  else if(cmd.startsWith("SCAN_RANGE:")){
    int p1=cmd.indexOf(':',11);
    int p2=cmd.indexOf(':',p1+1);
    if(p1>0 && p2>0){
      scanStart=cmd.substring(11,p1).toFloat();
      scanEnd=cmd.substring(p1+1,p2).toFloat();
      scanStep=cmd.substring(p2+1).toFloat();
      if(scanStep<0.05) scanStep=0.05;
      currentFreq=scanStart;
      scanning=true; sdrActive=false; recState=0;
      sendStatus("اسکن از "+String(scanStart)+" تا "+String(scanEnd));
    }
  }
  else if(cmd=="STOP"){ scanning=false; if(recState)finishRecording(); sendStatus("متوقف"); }
  else if(cmd=="LIST"){ sendList(); }
  else if(cmd=="GET_SETTINGS"){ sendSettings(); }
  else if(cmd=="CLEAR_ALL"){ clearAllSignals(); }
  else if(cmd=="TOGGLE_OOK"){
    cfgOOK=!cfgOOK; savePrefs(); reinitRadio();
    ws.textAll("{\"type\":\"OOK\",\"on\":"+String(cfgOOK?"true":"false")+"}");
  }
  else if(cmd.startsWith("REC:")){
    int c1=cmd.indexOf(':',4);
    if(c1>0){
      float f=cmd.substring(4,c1).toFloat();
      String n=cmd.substring(c1+1);
      if(f<240||f>930){ sendStatus("❌ فرکانس نامعتبر"); return; }
      scanning=false; sdrActive=false;
      startRecording(f,n.c_str());
    }
  }
  else if(cmd.startsWith("PLAY:")){ replaySignal(cmd.substring(5).toInt()); }
  else if(cmd.startsWith("DEL:")){ deleteSignal(cmd.substring(4).toInt()); }
  else if(cmd.startsWith("SET_BITRATE:")){
    float br=cmd.substring(12).toFloat();
    if(br>=0.5&&br<=128){
      cfgBitrate=br; radio.setBaudRate(br); savePrefs();
      ws.textAll("{\"type\":\"BITRATE\",\"value\":"+String(br,1)+"}");
    }
  }
  else if(cmd.startsWith("THRESH:")){
    cfgThreshold=cmd.substring(7).toInt(); savePrefs();
  }
  else if(cmd.startsWith("SET_FREQ:")){
    float f=cmd.substring(9).toFloat();
    if(f>=240&&f<=930){ cfgFreq=f; radio.setFrequency(f); savePrefs(); }
  }
  else if(cmd=="SDR_START"){
    scanning=false; recState=0;
    sdrActive=true; sdrBufIdx=0; lastSdrSample=micros();
    radio.setFrequency(cfgFreq);
    radio.startListening();
    delay(30);
    ws.textAll("{\"type\":\"SDR_STATUS\",\"msg\":\"در حال پخش صدا\"}");
    ws.textAll("{\"type\":\"SDR_FREQ\",\"freq\":"+String(cfgFreq,2)+"}");
  }
  else if(cmd=="SDR_STOP"){
    sdrActive=false;
    ws.textAll("{\"type\":\"SDR_STATUS\",\"msg\":\"متوقف\"}");
  }
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
    char buf[80];
    snprintf(buf,sizeof(buf),"{\"type\":\"SDR_STATUS\",\"msg\":\"بهترین: %.2f MHz (%d dBm)\"}",bestF,bestR);
    ws.textAll(buf);
  }
}

// ==================== SDR WebSocket ====================
void onSdrWsEvent(AsyncWebSocket* s,AsyncWebSocketClient* c,AwsEventType t,void* a,uint8_t* d,size_t l){
  // send only
}

// ==================== Setup / Loop ====================
void setup(){
  Serial.begin(115200); delay(500);
  Serial.println("\n=== Si4432 Pro (Analyzer + SDR) ===");

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
    if(scanning) doScan();
    if(recState) doRecord();
    delay(1);
  }
}
