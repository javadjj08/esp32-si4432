// ============================================================
// HIZMOS Complete Firmware v6.0
// ESP32 + NRF24L01 + SI4432 + Browser Audio + Remote Capture
// + WiFi Scan, BLE Scan, Spectrum, Signal Gen, MQTT, WebSocket
// ============================================================
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RF24.h>
#include <RadioLib.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <NimBLEDevice.h>
#include <SD.h>
#include <PubSubClient.h>
#include <WebSocketsServer.h>

// ========== پین‌ها ==========
#define NRF_CE          21
#define NRF_CSN         22
#define SI4432_CS       27
#define SI4432_IRQ      35
#define SI4432_SDN      32
#define SI4432_RX_DATA  34
#define SD_CS           5
#define HSPI_SCK        14
#define HSPI_MISO       33
#define HSPI_MOSI       13

RF24 radio_nrf(NRF_CE, NRF_CSN);
Si4432 radio_si = new Module(SI4432_CS, SI4432_IRQ, SI4432_SDN);
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

bool nrf_ok = false;
bool si_ok = false;
bool sd_ok = false;

// ========== تنظیمات ==========
String cfg_device_name = "HIZMOS";
String cfg_ap_ssid = "HIZMOS-AP";
String cfg_ap_password = "hizmos123";
int cfg_nrf_data_rate = 250;
int cfg_nrf_pa_level = 2;
int cfg_nrf_channel = 76;
String cfg_nrf_address = "HIZ01";
float cfg_si_freq = 433.92;
float cfg_si_data_rate = 2.4;
int cfg_si_power = 10;
String cfg_si_modulation = "OOK";
int cfg_spec_start = 430;
int cfg_spec_end = 440;
int cfg_spec_step = 1;
int cfg_rssi_samples = 10;
String cfg_mqtt_server = "192.168.1.100";
int cfg_mqtt_port = 1883;

// ========== بافر ضبط خام ==========
#define RAW_MAX_PULSES 1024
uint16_t rawPulses[RAW_MAX_PULSES];
int rawPulseCount = 0;
bool rawCapturing = false;
uint32_t rawLastTime = 0;
bool rawLastState = false;
uint32_t rawCaptureStart = 0;
uint32_t rawCaptureDuration = 2000;

// ========== HTML Page (Embedded) ==========
const char HTML_PAGE[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="fa" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>HIZMOS v6</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Tahoma,Arial,sans-serif;background:#0a0a0f;color:#e0e0e0;padding:8px;font-size:13px}
.device{max-width:500px;margin:0 auto;background:#12121a;border-radius:14px;padding:12px;border:1px solid #1a1a2e}
.header{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;padding-bottom:8px;border-bottom:1px solid #1a1a2e}
.logo{font-size:1.05rem;font-weight:bold;color:#00d4ff}
.led{width:12px;height:12px;border-radius:50%;background:#333}
.led.green{background:#0f0;box-shadow:0 0 8px #0f0}
.led.red{background:#f00;box-shadow:0 0 8px #f00}
.oled{background:#000;border-radius:8px;padding:10px;margin-bottom:10px;border:1px solid #1a1a2e;min-height:200px}
.menu-item{display:flex;justify-content:space-between;align-items:center;padding:9px 10px;margin:3px 0;border-radius:6px;cursor:pointer;font-size:0.85rem}
.menu-item:hover{background:#1a1a2e}
.menu-item.sel{background:#00d4ff;color:#000;font-weight:bold}
.badge{font-size:0.65rem;padding:2px 7px;border-radius:4px;background:#333;color:#fff}
.badge.ok{background:#0a5}.badge.na{background:#a00}.badge.edu{background:#06c}
.nav{display:flex;gap:6px;margin-top:10px;justify-content:center}
.nav button{background:#1a1a2e;border:1px solid #2a2a4e;color:#fff;padding:10px 16px;border-radius:8px;font-size:0.82rem;cursor:pointer}
.nav button:hover{background:#2a2a4e}.nav button.sel{background:#00d4ff;color:#000;font-weight:bold}
.status{text-align:center;font-size:0.75rem;color:#888;margin-top:6px}
.view{display:none;padding:6px 0}.view.on{display:block}
.back{background:#2a2a4e;border:none;color:#fff;padding:7px 14px;border-radius:6px;cursor:pointer;margin-bottom:8px}
h3{color:#00d4ff;margin:8px 0 6px;font-size:0.9rem}
.note{background:#0a1a0a;border:1px solid #0a5;border-radius:6px;padding:7px;font-size:0.73rem;color:#8f8;margin:5px 0;line-height:1.5}
.btn{display:block;width:100%;padding:10px;margin:4px 0;border:none;border-radius:6px;background:#1a1a2e;color:#fff;font-size:0.83rem;cursor:pointer;text-align:right}
.btn:hover{background:#2a2a4e}.btn.info{background:#06c}.btn.green{background:#0a5}.btn.red{background:#a00}.btn.warn{background:#a80}
.log{background:#0a0a12;border:1px solid #1a1a2e;border-radius:6px;padding:8px;font-size:0.7rem;max-height:220px;overflow-y:auto;direction:ltr;text-align:left;margin-top:6px;font-family:monospace;white-space:pre-wrap;color:#8f8;line-height:1.4}
canvas{width:100%;height:180px;background:#000;border-radius:6px;border:1px solid #1a1a2e;margin:6px 0}
.meter{height:22px;background:#0a0a12;border-radius:4px;overflow:hidden;border:1px solid #1a1a2e;margin:6px 0;position:relative}
.meter-fill{height:100%;background:linear-gradient(90deg,#0a5,#ff0,#a00);width:0%;transition:width 0.2s}
.meter-text{position:absolute;top:0;left:0;right:0;bottom:0;text-align:center;line-height:22px;font-size:0.72rem;font-weight:bold;color:#fff}
textarea{width:100%;height:180px;background:#0a0a12;color:#0f0;border:1px solid #1a1a2e;border-radius:6px;padding:8px;font-family:monospace;font-size:0.7rem;direction:ltr}
input[type=file],input[type=number],input[type=text],select{background:#1a1a2e;color:#fff;border:1px solid #2a2a4e;border-radius:5px;padding:7px;font-size:0.8rem;width:100%;margin:4px 0}
label{font-size:0.78rem;color:#aaa;display:block;margin-top:6px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.playing{background:#0a5 !important;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.6}}
</style>
</head>
<body>
<div class="device">
  <div class="header"><span class="logo" id="dName">HIZMOS v6</span><div class="led" id="led"></div></div>
  <div class="oled" id="oled"><div id="menu"></div></div>
  <div class="nav" id="nav">
    <button onclick="up()">UP</button><button class="sel" onclick="sel()">SELECT</button><button onclick="down()">DOWN</button>
  </div>
  <div class="status" id="st">Initializing...</div>

  <div class="view" id="v_nrf">
    <button class="back" onclick="home()">بازگشت</button>
    <h3>📡 NRF24L01 (2.4GHz)</h3>
    <button class="btn info" onclick="cmd('/nrf_info')">اطلاعات کانال</button>
    <button class="btn green" onclick="cmd('/nrf_ping')">ارسال تست</button>
    <button class="btn info" onclick="cmd('/nrf_scan')">اسکن کانال</button>
    <div class="log" id="l_nrf"></div>
  </div>

  <div class="view" id="v_si">
    <button class="back" onclick="home()">بازگشت</button>
    <h3>📻 SI4432 Sub-GHz</h3>
    <button class="btn info" onclick="show('v_spec')">📊 طیف زنده</button>
    <button class="btn info" onclick="show('v_rssi')">📶 RSSI زنده</button>
    <button class="btn info" onclick="cmd('/si_spectrum')">اسکن متنی</button>
    <button class="btn info" onclick="show('v_mod')">🎛️ مدولاسیون</button>
    <button class="btn info" onclick="show('v_dr')">⚡ نرخ داده</button>
    <button class="btn info" onclick="show('v_siggen')">📡 مولد سیگنال</button>
    <div class="log" id="l_si"></div>
  </div>

  <div class="view" id="v_spec">
    <button class="back" onclick="show('v_si')">بازگشت</button>
    <h3>طیف زنده</h3>
    <canvas id="specChart"></canvas>
    <div class="grid2">
      <button class="btn green" onclick="startSpec()">شروع</button>
      <button class="btn red" onclick="stopSpec()">توقف</button>
    </div>
    <button class="btn info" onclick="cmd('/si_spectrum_save')">ذخیره CSV</button>
  </div>

  <div class="view" id="v_rssi">
    <button class="back" onclick="show('v_si')">بازگشت</button>
    <h3>RSSI زنده</h3>
    <div class="meter"><div class="meter-fill" id="mFill"></div><div class="meter-text" id="mText">-- dBm</div></div>
    <div class="grid2">
      <button class="btn green" onclick="startRSSI()">شروع</button>
      <button class="btn red" onclick="stopRSSI()">توقف</button>
    </div>
    <div class="log" id="l_rssi"></div>
  </div>

  <div class="view" id="v_mod">
    <button class="back" onclick="show('v_si')">بازگشت</button>
    <h3>مدولاسیون</h3>
    <select id="modSel"><option>FSK</option><option>GFSK</option><option selected>OOK</option></select>
    <button class="btn green" onclick="applyMod()">اعمال</button>
    <div class="log" id="l_mod"></div>
  </div>

  <div class="view" id="v_dr">
    <button class="back" onclick="show('v_si')">بازگشت</button>
    <h3>نرخ داده</h3>
    <select id="drSel">
      <option value="1.2">1.2 kbps</option><option value="2.4" selected>2.4 kbps</option>
      <option value="4.8">4.8 kbps</option><option value="9.6">9.6 kbps</option>
      <option value="19.2">19.2 kbps</option><option value="38.4">38.4 kbps</option>
    </select>
    <button class="btn green" onclick="applyDR()">اعمال</button>
    <div class="log" id="l_dr"></div>
  </div>

  <div class="view" id="v_siggen">
    <button class="back" onclick="show('v_si')">بازگشت</button>
    <h3>📡 مولد سیگنال OOK</h3>
    <label>فرکانس (MHz):</label>
    <input type="number" id="genFreq" value="433.92" step="0.01">
    <label>الگوی OOK (مثلاً 1010):</label>
    <input type="text" id="genPattern" value="1010101010101010">
    <button class="btn green" onclick="startSigGen()">شروع ارسال</button>
    <button class="btn red" onclick="stopSigGen()">توقف</button>
    <div class="log" id="l_siggen"></div>
  </div>

  <div class="view" id="v_listen">
    <button class="back" onclick="home()">بازگشت</button>
    <h3>🔊 شنیدن با اسپیکر گوشی</h3>
    <div class="note">
      سیگنال از RF گرفته می‌شود و از طریق اسپیکر گوشی پخش می‌شود.<br>
      نیازی به بلندگوی خارجی نیست.
    </div>
    <label>فرکانس (MHz):</label>
    <input type="number" id="lstFreq" value="433.92" step="0.01">
    <button class="btn green" id="btnListen" onclick="toggleListen()">▶️ شروع شنیدن</button>
    <button class="btn info" onclick="singleListen()">🔊 پخش یک‌باره (۲ ثانیه)</button>
    <div class="log" id="l_listen"></div>
  </div>

  <div class="view" id="v_raw">
    <button class="back" onclick="home()">بازگشت</button>
    <h3>📡 کپی ریموت ۴۳۳MHz</h3>
    <div class="note">
      ریموت‌های کد ثابت (PT2262/EV1527) قابل کپی هستند.<br>
      می‌توانید سیگنال ضبط‌شده را با اسپیکر گوشی بشنوید.
    </div>
    <label>فرکانس ریموت (MHz):</label>
    <input type="number" id="rawFreq" value="433.92" step="0.01">
    <button class="btn green" onclick="rawCaptureAndPlay()">🔴 ضبط + پخش با گوشی</button>
    <button class="btn info" onclick="rawCaptureOnly()">🎙️ فقط ضبط (۲ ثانیه)</button>
    <button class="btn info" onclick="playLastCapture()">🔊 پخش آخرین ضبط</button>
    <button class="btn green" onclick="rawReplayRF()">📡 بازپخش با RF</button>
    <div class="grid2">
      <button class="btn info" onclick="cmd('/raw_save')">💾 ذخیره</button>
      <button class="btn info" onclick="cmd('/raw_load')">📂 بارگذاری</button>
    </div>
    <button class="btn red" onclick="cmd('/raw_clear')">🗑 پاک کردن</button>
    <div class="log" id="l_raw"></div>
  </div>

  <div class="view" id="v_wifi">
    <button class="back" onclick="home()">بازگشت</button>
    <h3>📶 اسکن WiFi</h3>
    <button class="btn green" onclick="cmd('/wifi_scan')">شروع اسکن</button>
    <div class="log" id="l_wifi"></div>
  </div>

  <div class="view" id="v_ble">
    <button class="back" onclick="home()">بازگشت</button>
    <h3>🔵 اسکن Bluetooth</h3>
    <button class="btn green" onclick="cmd('/ble_scan')">شروع اسکن</button>
    <div class="log" id="l_ble"></div>
  </div>

  <div class="view" id="v_mqtt">
    <button class="back" onclick="home()">بازگشت</button>
    <h3>🌐 MQTT</h3>
    <button class="btn info" onclick="cmd('/mqtt_status')">وضعیت اتصال</button>
    <button class="btn green" onclick="cmd('/mqtt_publish')">انتشار تست</button>
    <div class="log" id="l_mqtt"></div>
  </div>

  <div class="view" id="v_cfg">
    <button class="back" onclick="home()">بازگشت</button>
    <h3>⚙️ تنظیمات</h3>
    <input type="file" id="cfgFile" accept=".json,.txt" onchange="loadFile(event)">
    <textarea id="cfgText"></textarea>
    <button class="btn green" onclick="saveConfig()">💾 ذخیره و اعمال</button>
    <button class="btn info" onclick="reloadConfig()">🔄 بارگذاری</button>
    <button class="btn red" onclick="resetConfig()">↺ پیش‌فرض</button>
    <div class="log" id="l_cfg"></div>
  </div>

  <div class="view" id="v_ota">
    <button class="back" onclick="home()">بازگشت</button>
    <h3>📦 OTA Update</h3>
    <form method="POST" action="/update" enctype="multipart/form-data">
      <input type="file" name="firmware" accept=".bin" required>
      <button class="btn green" type="submit">📤 آپلود فریمور</button>
    </form>
  </div>

  <div class="view" id="v_sys">
    <button class="back" onclick="home()">بازگشت</button>
    <h3>ℹ️ سیستم</h3>
    <button class="btn info" onclick="cmd('/sys_info')">اطلاعات سیستم</button>
    <button class="btn warn" onclick="reboot()">🔄 ری‌استارت</button>
    <div class="log" id="l_sys"></div>
  </div>
</div>

<script>
var audioCtx = null;
function initAudio() {
  if (!audioCtx) {
    try {
      audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    } catch(e) { log2('AudioContext not supported'); return null; }
  }
  if (audioCtx.state === 'suspended') audioCtx.resume();
  return audioCtx;
}

function playPulses(pulses, freqHz) {
  var ctx = initAudio();
  if (!ctx || !pulses || pulses.length === 0) return 0;
  freqHz = freqHz || 2000;
  var sr = ctx.sampleRate;
  var totalUs = 0;
  for (var i = 0; i < pulses.length; i++) totalUs += pulses[i];
  var totalSec = totalUs / 1000000;
  if (totalSec < 0.001) return 0;

  var samples = Math.ceil(totalSec * sr);
  var buffer = ctx.createBuffer(1, samples, sr);
  var data = buffer.getChannelData(0);

  var offset = 0;
  var state = true;
  var phase = 0;
  var dphase = 2 * Math.PI * freqHz / sr;

  for (var i = 0; i < pulses.length; i++) {
    var dur = Math.round(pulses[i] * sr / 1000000);
    if (state) {
      for (var j = 0; j < dur && offset + j < samples; j++) {
        data[offset + j] = Math.sin(phase) * 0.4;
        phase += dphase;
        if (phase > 2 * Math.PI) phase -= 2 * Math.PI;
      }
    }
    offset += dur;
    state = !state;
  }

  var source = ctx.createBufferSource();
  source.buffer = buffer;
  source.connect(ctx.destination);
  source.start();
  return totalSec;
}

var items = [
  {id:'nrf',label:'📡 NRF24L01 (2.4GHz)',v:'v_nrf',en:true},
  {id:'si',label:'📻 SI4432 Sub-GHz',v:'v_si',en:true},
  {id:'listen',label:'🔊 شنیدن با اسپیکر گوشی',v:'v_listen',en:true},
  {id:'raw',label:'📡 کپی ریموت',v:'v_raw',en:true},
  {id:'wifi',label:'📶 اسکن WiFi',v:'v_wifi',en:true},
  {id:'ble',label:'🔵 اسکن Bluetooth',v:'v_ble',en:true},
  {id:'mqtt',label:'🌐 MQTT',v:'v_mqtt',en:true},
  {id:'cfg',label:'⚙️ تنظیمات',v:'v_cfg',en:true},
  {id:'ota',label:'📦 OTA Update',v:'v_ota',en:true},
  {id:'sys',label:'ℹ️ سیستم',v:'v_sys',en:true}
];
var status={nrf:'N/A',si:'N/A'},cur=0,inSub=false;
var specTimer=null,rssiTimer=null,specCtx=null;
var listening=false, listenTimer=null, lastPulses=null;
var ws=null;

function initWS(){
  try{
    ws=new WebSocket('ws://'+location.hostname+':81/');
    ws.onmessage=function(e){
      var d=JSON.parse(e.data);
      if(d.type==='log'){ log2(d.msg); }
      else if(d.type==='spec'){ drawChart(d.data); }
    };
    ws.onclose=function(){ setTimeout(initWS,3000); };
  }catch(e){ console.log('WS error',e); }
}

function build(){var m=document.getElementById('menu'),h='';for(var i=0;i<items.length;i++){var it=items[i];var st=(it.id==='nrf')?status.nrf:(it.id==='si')?status.si:'EDU';var cls='badge '+((st==='OK')?'ok':(st==='N/A')?'na':'edu');var s=(i===cur&&!inSub)?' sel':'';h+='<div class="menu-item'+s+'" onclick="pick('+i+')"><span>'+it.label+'</span><span class="'+cls+'">'+st+'</span></div>';}m.innerHTML=h;}
function pick(i){cur=i;sel();}
function up(){if(inSub)return;do{cur=(cur-1+items.length)%items.length;}while(!items[cur].en);build();}
function down(){if(inSub)return;do{cur=(cur+1)%items.length;}while(!items[cur].en);build();}
function sel(){if(inSub)return;show(items[cur].v);}
function show(id){var vs=document.getElementsByClassName('view');for(var i=0;i<vs.length;i++)vs[i].className='view';var el=document.getElementById(id);if(el)el.className='view on';document.getElementById('oled').style.display='none';document.getElementById('nav').style.display='none';inSub=true;if(id==='v_spec')initChart();}
function home(){stopSpec();stopRSSI();stopListen();stopSigGen();var vs=document.getElementsByClassName('view');for(var i=0;i<vs.length;i++)vs[i].className='view';document.getElementById('oled').style.display='block';document.getElementById('nav').style.display='flex';inSub=false;refresh();}
function cmd(url){fetch(url).then(r=>r.text()).then(t=>{var a=document.querySelector('.view.on .log');if(a){a.innerHTML+=t+'\n';a.scrollTop=a.scrollHeight;}}).catch(()=>{});}
function log2(msg){var a=document.querySelector('.view.on .log');if(a){a.innerHTML+=msg+'\n';a.scrollTop=a.scrollHeight;}}
function refresh(){fetch('/status').then(r=>r.json()).then(d=>{status=d;document.getElementById('led').className='led '+((d.nrf==='OK'||d.si==='OK')?'green':'red');document.getElementById('st').innerText='NRF24: '+d.nrf+' | SI4432: '+d.si;if(d.device_name)document.getElementById('dName').innerText=d.device_name;build();}).catch(()=>{});}

function initChart(){var c=document.getElementById('specChart');if(!c)return;specCtx=c.getContext('2d');c.width=c.offsetWidth;c.height=c.offsetHeight;drawChart([]);}
function drawChart(data){if(!specCtx)return;var c=specCtx.canvas;specCtx.fillStyle='#000';specCtx.fillRect(0,0,c.width,c.height);specCtx.strokeStyle='#1a3a4a';for(var i=1;i<10;i++){var y=(c.height/10)*i;specCtx.beginPath();specCtx.moveTo(0,y);specCtx.lineTo(c.width,y);specCtx.stroke();}if(data.length===0)return;var bw=c.width/data.length;for(var i=0;i<data.length;i++){var pct=(data[i]+120)/90;if(pct<0)pct=0;if(pct>1)pct=1;var h=pct*c.height;specCtx.fillStyle='rgb('+Math.floor(255*pct)+','+Math.floor(255*(1-pct))+',50)';specCtx.fillRect(i*bw,c.height-h,bw-1,h);}}
function startSpec(){stopSpec();initChart();specTimer=setInterval(()=>{fetch('/si_spectrum_json').then(r=>r.json()).then(d=>drawChart(d.rssi)).catch(()=>{});},1500);}
function stopSpec(){if(specTimer){clearInterval(specTimer);specTimer=null;}}

function startRSSI(){stopRSSI();rssiTimer=setInterval(()=>{fetch('/si_rssi_value').then(r=>r.json()).then(d=>{var m=document.getElementById('mFill'),t=document.getElementById('mText');if(!m||!t)return;var pct=(d.rssi+120)/90*100;if(pct<0)pct=0;if(pct>100)pct=100;m.style.width=pct+'%';t.innerText=d.rssi.toFixed(1)+' dBm';}).catch(()=>{});},500);}
function stopRSSI(){if(rssiTimer){clearInterval(rssiTimer);rssiTimer=null;}}

function applyMod(){var v=document.getElementById('modSel').value;fetch('/si_set_mod?mod='+v).then(r=>r.text()).then(t=>log2(t));}
function applyDR(){var v=document.getElementById('drSel').value;fetch('/si_set_dr?dr='+v).then(r=>r.text()).then(t=>log2(t));}

function startSigGen(){
  var f=document.getElementById('genFreq').value;
  var p=document.getElementById('genPattern').value;
  fetch('/siggen_start?freq='+f+'&pattern='+p).then(r=>r.text()).then(t=>log2(t));
}
function stopSigGen(){fetch('/siggen_stop').then(r=>r.text()).then(t=>log2(t));}

function toggleListen(){if(listening){stopListen();}else{startListen();}}
function startListen(){
  initAudio();
  var f = document.getElementById('lstFreq').value;
  fetch('/si_set_freq?freq='+f).then(()=>{
    listening = true;
    var btn = document.getElementById('btnListen');
    btn.innerText = '⏹ توقف شنیدن';
    btn.className = 'btn red playing';
    log2('🎧 Listening...');
    listenLoop();
  });
}
function stopListen(){
  listening = false;
  if(listenTimer){ clearTimeout(listenTimer); listenTimer=null; }
  var btn = document.getElementById('btnListen');
  if(btn){ btn.innerText = '▶️ شروع شنیدن'; btn.className = 'btn green'; }
}
async function listenLoop(){
  while(listening){
    try {
      await fetch('/raw_capture');
      await sleep(2200);
      var r = await fetch('/raw_pulses');
      var d = await r.json();
      if(d.pulses && d.pulses.length > 0){
        var dur = playPulses(d.pulses, 2500);
        log2('▶ Played ' + d.pulses.length + ' pulses (' + dur.toFixed(2) + 's)');
        await sleep(dur * 1000);
      } else {
        await sleep(300);
      }
    } catch(e) {
      log2('Error: ' + e);
      await sleep(500);
    }
  }
}
function singleListen(){
  initAudio();
  var f = document.getElementById('lstFreq').value;
  fetch('/si_set_freq?freq='+f).then(()=>{
    fetch('/raw_capture').then(()=>{
      log2('🎙️ Recording 2s...');
      setTimeout(()=>{
        fetch('/raw_pulses').then(r=>r.json()).then(d=>{
          if(d.pulses && d.pulses.length > 0){
            var dur = playPulses(d.pulses, 2500);
            log2('▶ Played ' + d.pulses.length + ' pulses');
          } else log2('No signal captured');
        });
      }, 2200);
    });
  });
}
function sleep(ms){ return new Promise(r=>setTimeout(r,ms)); }

function rawCaptureAndPlay(){
  initAudio();
  var f = document.getElementById('rawFreq').value;
  fetch('/si_set_freq?freq='+f).then(()=>{
    fetch('/raw_capture').then(()=>{
      log2('🎙️ Recording 2s... Press remote NOW!');
      setTimeout(()=>{
        fetch('/raw_pulses').then(r=>r.json()).then(d=>{
          if(d.pulses && d.pulses.length > 0){
            lastPulses = d.pulses;
            var dur = playPulses(d.pulses, 2000);
            log2('▶ Captured ' + d.pulses.length + ' pulses, played (' + dur.toFixed(2) + 's)');
          } else log2('No signal captured');
        });
      }, 2200);
    });
  });
}
function rawCaptureOnly(){
  var f = document.getElementById('rawFreq').value;
  fetch('/si_set_freq?freq='+f).then(()=>{
    fetch('/raw_capture').then(()=>{
      log2('🎙️ Recording 2s...');
      setTimeout(()=>{
        fetch('/raw_pulses').then(r=>r.json()).then(d=>{
          lastPulses = d.pulses;
          log2('Captured ' + (d.pulses?d.pulses.length:0) + ' pulses');
        });
      }, 2200);
    });
  });
}
function playLastCapture(){
  initAudio();
  if(!lastPulses || lastPulses.length === 0){
    log2('No previous capture. Press record first.');
    return;
  }
  playPulses(lastPulses, 2000);
  log2('▶ Played last capture');
}
function rawReplayRF(){fetch('/raw_replay').then(r=>r.text()).then(t=>log2(t));}

function loadFile(ev){var f=ev.target.files[0];if(!f)return;var r=new FileReader();r.onload=e=>document.getElementById('cfgText').value=e.target.result;r.readAsText(f);}
function reloadConfig(){fetch('/config_get').then(r=>r.text()).then(t=>document.getElementById('cfgText').value=t);}
function saveConfig(){var t=document.getElementById('cfgText').value;fetch('/config_save',{method:'POST',headers:{'Content-Type':'application/json'},body:t}).then(r=>r.text()).then(x=>{log2(x);setTimeout(()=>location.reload(),2500);});}
function resetConfig(){fetch('/config_reset').then(r=>r.text()).then(x=>{log2(x);setTimeout(()=>location.reload(),2500);});}
function reboot(){if(confirm('ری‌استارت؟'))fetch('/sys_reboot');}

build();refresh();setInterval(refresh,5000);initWS();
</script>
</body>
</html>
)HTMLPAGE";

// ========== Config ==========
void applyConfig(const String& json) {
  StaticJsonDocument<4096> doc;
  if (deserializeJson(doc, json)) { Serial.println("JSON Error"); return; }
  cfg_device_name = doc["device_name"] | "HIZMOS";
  cfg_ap_ssid     = doc["ap_ssid"]     | "HIZMOS-AP";
  cfg_ap_password = doc["ap_password"] | "hizmos123";
  if (doc.containsKey("nrf")) {
    JsonObject n = doc["nrf"];
    cfg_nrf_data_rate = n["data_rate"] | 250;
    cfg_nrf_pa_level  = n["pa_level"]  | 2;
    cfg_nrf_channel   = n["channel"]   | 76;
    cfg_nrf_address   = String((const char*)(n["address"] | "HIZ01"));
  }
  if (doc.containsKey("si4432")) {
    JsonObject s = doc["si4432"];
    cfg_si_freq       = s["frequency"]    | 433.92;
    cfg_si_data_rate  = s["data_rate"]    | 2.4;
    cfg_si_power      = s["output_power"] | 10;
    cfg_si_modulation = String((const char*)(s["modulation"] | "OOK"));
    if (s.containsKey("spectrum")) {
      cfg_spec_start = s["spectrum"]["start_freq"] | 430;
      cfg_spec_end   = s["spectrum"]["end_freq"]   | 440;
      cfg_spec_step  = s["spectrum"]["step"]       | 1;
    }
  }
  if (doc.containsKey("rssi")) cfg_rssi_samples = doc["rssi"]["samples"] | 10;
  if (doc.containsKey("mqtt")) {
    cfg_mqtt_server = String((const char*)(doc["mqtt"]["server"] | "192.168.1.100"));
    cfg_mqtt_port   = doc["mqtt"]["port"] | 1883;
  }
  Serial.println("Config applied");
}

String buildDefaultConfig() {
  String j = "{";
  j += "\"device_name\":\"HIZMOS\",\"ap_ssid\":\"HIZMOS-AP\",\"ap_password\":\"hizmos123\",";
  j += "\"nrf\":{\"data_rate\":250,\"pa_level\":2,\"channel\":76,\"address\":\"HIZ01\"},";
  j += "\"si4432\":{\"frequency\":433.92,\"data_rate\":2.4,\"output_power\":10,";
  j += "\"modulation\":\"OOK\",\"spectrum\":{\"start_freq\":430,\"end_freq\":440,\"step\":1}},";
  j += "\"rssi\":{\"samples\":10},";
  j += "\"mqtt\":{\"server\":\"192.168.1.100\",\"port\":1883}}";
  return j;
}

void loadConfig() {
  if (!LittleFS.begin(true)) return;
  if (!LittleFS.exists("/config.json")) {
    File f = LittleFS.open("/config.json", "w");
    f.print(buildDefaultConfig()); f.close();
  }
  File f = LittleFS.open("/config.json", "r");
  if (f) { String c = f.readString(); f.close(); applyConfig(c); }
}

// ========== MQTT ==========
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT msg: "); Serial.println(topic);
}

void mqttReconnect() {
  if (!mqttClient.connected()) {
    String clientId = "HIZMOS-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      mqttClient.subscribe("hizmos/cmd");
      Serial.println("MQTT connected");
    }
  }
}

void mqttPublish(const String& topic, const String& payload) {
  if (mqttClient.connected()) mqttClient.publish(topic.c_str(), payload.c_str());
}

// ========== Detection ==========
bool detect_nrf() {
  if (radio_nrf.begin()) {
    radio_nrf.openWritingPipe((const byte*)cfg_nrf_address.c_str());
    radio_nrf.setPALevel(cfg_nrf_pa_level);
    rf24_datarate_e dr = (cfg_nrf_data_rate == 250) ? RF24_250KBPS :
                         (cfg_nrf_data_rate == 1000) ? RF24_1MBPS : RF24_2MBPS;
    radio_nrf.setDataRate(dr);
    radio_nrf.setChannel(cfg_nrf_channel);
    return true;
  }
  return false;
}

bool detect_si4432() {
  SPI.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, SI4432_CS);
  int st = radio_si.begin(cfg_si_freq);
  if (st == RADIOLIB_ERR_NONE) {
    radio_si.setOutputPower(cfg_si_power);
    if (cfg_si_modulation == "OOK") radio_si.setModulation(RADIOLIB_SI443X_MODULATION_OOK);
    else if (cfg_si_modulation == "GFSK") radio_si.setModulation(RADIOLIB_SI443X_MODULATION_GFSK);
    else radio_si.setModulation(RADIOLIB_SI443X_MODULATION_FSK);
    return true;
  }
  return false;
}

bool detect_sd() {
  if (SD.begin(SD_CS)) return true;
  return false;
}

// ========== Raw Capture ==========
void rawStartCapture() {
  rawPulseCount = 0;
  rawCapturing = true;
  rawCaptureStart = millis();
  rawLastState = digitalRead(SI4432_RX_DATA);
  rawLastTime = micros();
}

void rawProcessCapture() {
  if (!rawCapturing) return;
  if (millis() - rawCaptureStart > rawCaptureDuration) {
    rawCapturing = false;
    return;
  }
  bool cur = digitalRead(SI4432_RX_DATA);
  if (cur != rawLastState) {
    uint32_t now = micros();
    uint32_t dur = now - rawLastTime;
    if (dur > 30 && rawPulseCount < RAW_MAX_PULSES) {
      rawPulses[rawPulseCount++] = (uint16_t)min(dur, (uint32_t)65535);
      rawLastTime = now;
      rawLastState = cur;
    }
  }
}

void rawReplaySignal() {
  if (rawPulseCount == 0 || !si_ok) return;
  radio_si.setFrequency(cfg_si_freq);
  radio_si.setBitRate(2.4);
  radio_si.transmitDirect();
  bool state = true;
  for (int i = 0; i < rawPulseCount; i++) {
    digitalWrite(SI4432_SDN, state ? HIGH : LOW);
    delayMicroseconds(rawPulses[i]);
    state = !state;
  }
  digitalWrite(SI4432_SDN, LOW);
  radio_si.standby();
}

// ========== Signal Generator ==========
bool sigGenActive = false;
String sigGenPattern = "1010101010101010";
float sigGenFreq = 433.92;
int sigGenIndex = 0;

void startSigGen(float freq, const String& pattern) {
  if (!si_ok) return;
  sigGenFreq = freq;
  sigGenPattern = pattern;
  sigGenIndex = 0;
  sigGenActive = true;
  radio_si.setFrequency(sigGenFreq);
  radio_si.setBitRate(2.4);
  radio_si.transmitDirect();
}

void stopSigGen() {
  sigGenActive = false;
  digitalWrite(SI4432_SDN, LOW);
  if (si_ok) radio_si.standby();
}

void processSigGen() {
  if (!sigGenActive || sigGenPattern.length() == 0) return;
  char c = sigGenPattern[sigGenIndex];
  digitalWrite(SI4432_SDN, c == '1' ? HIGH : LOW);
  delayMicroseconds(500);
  sigGenIndex = (sigGenIndex + 1) % sigGenPattern.length();
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HIZMOS v6.0 ===");

  pinMode(SI4432_RX_DATA, INPUT);
  pinMode(SI4432_SDN, OUTPUT);
  digitalWrite(SI4432_SDN, LOW);

  loadConfig();

  nrf_ok = detect_nrf();
  si_ok  = detect_si4432();
  sd_ok  = detect_sd();
  Serial.printf("[NRF24] %s  [SI4432] %s  [SD] %s\n", nrf_ok?"OK":"N/A", si_ok?"OK":"N/A", sd_ok?"OK":"N/A");

  WiFi.softAP(cfg_ap_ssid.c_str(), cfg_ap_password.c_str());
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  // MQTT
  mqttClient.setServer(cfg_mqtt_server.c_str(), cfg_mqtt_port);
  mqttClient.setCallback(mqttCallback);

  // WebSocket
  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
      String msg = String((char*)payload);
      Serial.printf("WS[%u]: %s\n", num, msg.c_str());
    }
  });

  // BLE
  NimBLEDevice::init("HIZMOS-BLE");

  server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html", HTML_PAGE); });
  server.on("/status", HTTP_GET, [](){
    String j = "{\"nrf\":\"" + String(nrf_ok?"OK":"N/A") + "\",\"si\":\"" + String(si_ok?"OK":"N/A") + "\",\"device_name\":\"" + cfg_device_name + "\"}";
    server.send(200, "application/json", j);
  });

  // ... (تمام endpointهای قبلی)
  server.on("/nrf_info", HTTP_GET, [](){
    if(!nrf_ok){ server.send(200,"text/plain","NRF24 N/A\n"); return; }
    String r = "=== NRF24 ===\nChannel: " + String(cfg_nrf_channel) + "\nAddress: " + cfg_nrf_address + "\nRate: " + String(cfg_nrf_data_rate) + " kbps\n";
    server.send(200,"text/plain", r);
  });
  server.on("/nrf_ping", HTTP_GET, [](){
    if(!nrf_ok){ server.send(200,"text/plain","NRF24 N/A\n"); return; }
    String r = "=== Ping ===\n";
    for(int i=1; i<=10; i++){
      String msg = "PKT-" + String(i);
      bool ok = radio_nrf.write(msg.c_str(), msg.length());
      r += "Packet " + String(i) + ": " + (ok?"OK":"FAIL") + "\n";
      delay(50);
    }
    server.send(200,"text/plain", r);
  });
  server.on("/nrf_scan", HTTP_GET, [](){
    if(!nrf_ok){ server.send(200,"text/plain","NRF24 N/A\n"); return; }
    String r = "=== Scan ===\n";
    for(int ch=0; ch<=125; ch+=25){ radio_nrf.setChannel(ch); delay(10); r += "CH " + String(ch) + " scanned\n"; }
    radio_nrf.setChannel(cfg_nrf_channel);
    server.send(200,"text/plain", r);
  });

  server.on("/si_set_freq", HTTP_GET, [](){
    if(server.hasArg("freq")){
      cfg_si_freq = server.arg("freq").toFloat();
      if(si_ok) radio_si.setFrequency(cfg_si_freq);
      server.send(200,"text/plain","Freq: " + String(cfg_si_freq,2));
    } else server.send(400,"text/plain","Missing");
  });
  server.on("/si_set_mod", HTTP_GET, [](){
    if(server.hasArg("mod")){
      cfg_si_modulation = server.arg("mod");
      if(si_ok) {
        if (cfg_si_modulation == "OOK") radio_si.setModulation(RADIOLIB_SI443X_MODULATION_OOK);
        else if (cfg_si_modulation == "GFSK") radio_si.setModulation(RADIOLIB_SI443X_MODULATION_GFSK);
        else radio_si.setModulation(RADIOLIB_SI443X_MODULATION_FSK);
      }
      server.send(200,"text/plain","Mod: " + cfg_si_modulation + "\n");
    } else server.send(400,"text/plain","Missing");
  });
  server.on("/si_set_dr", HTTP_GET, [](){
    if(server.hasArg("dr")){
      cfg_si_data_rate = server.arg("dr").toFloat();
      if(si_ok) radio_si.setBitRate(cfg_si_data_rate);
      server.send(200,"text/plain","DR: " + String(cfg_si_data_rate) + " kbps\n");
    } else server.send(400,"text/plain","Missing");
  });
  server.on("/si_spectrum", HTTP_GET, [](){
    if(!si_ok){ server.send(200,"text/plain","SI4432 N/A\n"); return; }
    String r = "=== Spectrum ===\n";
    for(int f=cfg_spec_start; f<=cfg_spec_end; f+=cfg_spec_step){
      radio_si.setFrequency(f); delay(30);
      r += String(f) + " MHz : " + String(radio_si.getRSSI()) + " dBm\n";
    }
    radio_si.setFrequency(cfg_si_freq);
    server.send(200,"text/plain", r);
  });
  server.on("/si_spectrum_json", HTTP_GET, [](){
    if(!si_ok){ server.send(200,"application/json","{\"rssi\":[]}"); return; }
    String j = "{\"rssi\":[";
    bool first = true;
    for(int f=cfg_spec_start; f<=cfg_spec_end; f+=cfg_spec_step){
      radio_si.setFrequency(f); delay(20);
      if(!first) j += ",";
      j += String(radio_si.getRSSI(), 1);
      first = false;
    }
    radio_si.setFrequency(cfg_si_freq);
    j += "]}";
    server.send(200,"application/json", j);
  });
  server.on("/si_spectrum_save", HTTP_GET, [](){
    if(!si_ok){ server.send(200,"text/plain","SI4432 N/A\n"); return; }
    String csv = "Freq_MHz,RSSI_dBm\n";
    for(int f=cfg_spec_start; f<=cfg_spec_end; f+=cfg_spec_step){
      radio_si.setFrequency(f); delay(20);
      csv += String(f) + "," + String(radio_si.getRSSI(),2) + "\n";
    }
    radio_si.setFrequency(cfg_si_freq);
    File file = LittleFS.open("/spectrum.csv","w");
    if(file){ file.print(csv); file.close(); server.send(200,"text/plain","Saved\n"); }
    else server.send(500,"text/plain","Failed\n");
  });
  server.on("/si_rssi", HTTP_GET, [](){
    if(!si_ok){ server.send(200,"text/plain","SI4432 N/A\n"); return; }
    String r = "=== RSSI ===\n";
    float sum = 0;
    for(int i=1; i<=cfg_rssi_samples; i++){
      float v = radio_si.getRSSI(); sum += v;
      r += "Sample " + String(i) + ": " + String(v) + " dBm\n";
      delay(150);
    }
    r += "\nAvg: " + String(sum/cfg_rssi_samples,2) + " dBm\n";
    server.send(200,"text/plain", r);
  });
  server.on("/si_rssi_value", HTTP_GET, [](){
    if(!si_ok){ server.send(200,"application/json","{\"rssi\":-120}"); return; }
    server.send(200,"application/json", "{\"rssi\":" + String(radio_si.getRSSI(),1) + "}");
  });

  server.on("/raw_capture", HTTP_GET, [](){
    if(!si_ok){ server.send(503,"text/plain","SI4432 N/A"); return; }
    radio_si.setFrequency(cfg_si_freq);
    radio_si.setBitRate(2.4);
    radio_si.receiveDirect();
    rawCaptureDuration = 2000;
    rawStartCapture();
    server.send(200,"text/plain","Recording 2s...");
  });

  server.on("/raw_pulses", HTTP_GET, [](){
    String j = "{\"count\":" + String(rawPulseCount) + ",\"freq\":" + String(cfg_si_freq,2) + ",\"pulses\":[";
    for(int i=0; i<rawPulseCount; i++){
      if(i>0) j += ",";
      j += String(rawPulses[i]);
    }
    j += "]}";
    server.send(200,"application/json", j);
  });

  server.on("/raw_replay", HTTP_GET, [](){
    if(!si_ok){ server.send(503,"text/plain","SI4432 N/A"); return; }
    if(rawPulseCount == 0){ server.send(400,"text/plain","No signal\n"); return; }
    rawReplaySignal();
    server.send(200,"text/plain","Replayed " + String(rawPulseCount) + " pulses via RF\n");
  });

  server.on("/raw_save", HTTP_GET, [](){
    if(rawPulseCount == 0){ server.send(400,"text/plain","No signal"); return; }
    StaticJsonDocument<4096> doc;
    doc["freq"] = cfg_si_freq;
    doc["count"] = rawPulseCount;
    JsonArray arr = doc.createNestedArray("pulses");
    for (int i = 0; i < rawPulseCount; i++) arr.add(rawPulses[i]);
    File f = LittleFS.open("/remote.json", "w");
    if(f){ serializeJson(doc, f); f.close(); server.send(200,"text/plain","Saved /remote.json\n"); }
    else server.send(500,"text/plain","Failed");
  });

  server.on("/raw_load", HTTP_GET, [](){
    if(!LittleFS.exists("/remote.json")){ server.send(404,"text/plain","No file"); return; }
    File f = LittleFS.open("/remote.json", "r");
    String c = f.readString(); f.close();
    StaticJsonDocument<4096> doc;
    if(deserializeJson(doc, c)){ server.send(400,"text/plain","Bad JSON"); return; }
    cfg_si_freq = doc["freq"] | 433.92;
    rawPulseCount = 0;
    for(JsonVariant v : doc["pulses"].as<JsonArray>()){
      if(rawPulseCount >= RAW_MAX_PULSES) break;
      rawPulses[rawPulseCount++] = v.as<uint16_t>();
    }
    server.send(200,"text/plain","Loaded " + String(rawPulseCount) + " pulses\n");
  });

  server.on("/raw_clear", HTTP_GET, [](){
    rawPulseCount = 0;
    server.send(200,"text/plain","Cleared\n");
  });

  // --- WiFi Scan ---
  server.on("/wifi_scan", HTTP_GET, [](){
    int n = WiFi.scanNetworks();
    String r = "=== WiFi Scan (" + String(n) + " networks) ===\n";
    for (int i = 0; i < n; i++) {
      r += String(i+1) + ". " + WiFi.SSID(i) + " (" + WiFi.RSSI(i) + " dBm) CH" + WiFi.channel(i) + "\n";
    }
    server.send(200,"text/plain", r);
  });

  // --- BLE Scan ---
  server.on("/ble_scan", HTTP_GET, [](){
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setActiveScan(true);
    NimBLEScanResults results = pScan->start(3);
    String r = "=== BLE Scan (" + String(results.getCount()) + " devices) ===\n";
    for (int i = 0; i < results.getCount(); i++) {
      NimBLEAdvertisedDevice dev = results.getDevice(i);
      r += dev.getAddress().toString().c_str() + String(" RSSI:") + String(dev.getRSSI()) + "\n";
    }
    server.send(200,"text/plain", r);
  });

  // --- Signal Generator ---
  server.on("/siggen_start", HTTP_GET, [](){
    if(server.hasArg("freq") && server.hasArg("pattern")){
      float f = server.arg("freq").toFloat();
      String p = server.arg("pattern");
      startSigGen(f, p);
      server.send(200,"text/plain","Signal Gen started\n");
    } else server.send(400,"text/plain","Missing args");
  });
  server.on("/siggen_stop", HTTP_GET, [](){
    stopSigGen();
    server.send(200,"text/plain","Signal Gen stopped\n");
  });

  // --- MQTT ---
  server.on("/mqtt_status", HTTP_GET, [](){
    String r = "MQTT: " + String(mqttClient.connected() ? "Connected" : "Disconnected") + "\n";
    r += "Server: " + cfg_mqtt_server + ":" + String(cfg_mqtt_port) + "\n";
    server.send(200,"text/plain", r);
  });
  server.on("/mqtt_publish", HTTP_GET, [](){
    mqttPublish("hizmos/status", "{\"device\":\"HIZMOS\",\"status\":\"ok\"}");
    server.send(200,"text/plain","Published\n");
  });

  // --- Config ---
  server.on("/config_get", HTTP_GET, [](){
    if(LittleFS.exists("/config.json")){
      File f = LittleFS.open("/config.json","r");
      String c = f.readString(); f.close();
      server.send(200,"application/json", c);
    } else server.send(200,"application/json", buildDefaultConfig());
  });
  server.on("/config_save", HTTP_POST, [](){
    if(server.hasArg("plain")){
      String body = server.arg("plain");
      if(body.length() > 0 && (uint8_t)body[0] == 0xEF) body = body.substring(3);
      body.trim();
      StaticJsonDocument<4096> td;
      if(deserializeJson(td, body)){ server.send(400,"text/plain","Invalid JSON"); return; }
      File f = LittleFS.open("/config.json","w");
      if(f){ f.print(body); f.close(); }
      applyConfig(body);
      server.send(200,"text/plain","Saved. Rebooting...");
      delay(2000); ESP.restart();
    } else server.send(400,"text/plain","No body");
  });
  server.on("/config_reset", HTTP_GET, [](){
    File f = LittleFS.open("/config.json","w");
    if(f){ f.print(buildDefaultConfig()); f.close(); }
    server.send(200,"text/plain","Reset. Rebooting...");
    delay(2000); ESP.restart();
  });

  // --- System ---
  server.on("/sys_info", HTTP_GET, [](){
    String r = "=== System ===\nChip: " + String(ESP.getChipModel()) + "\nCPU: " + String(ESP.getCpuFreqMHz()) + " MHz\nFree Heap: " + String(ESP.getFreeHeap()) + "\nUptime: " + String(millis()/1000) + " s\n";
    server.send(200,"text/plain", r);
  });
  server.on("/sys_reboot", HTTP_GET, [](){
    server.send(200,"text/plain","Rebooting...");
    delay(500); ESP.restart();
  });

  // --- OTA ---
  server.on("/update", HTTP_GET, [](){
    String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
    h += "<style>body{font-family:Tahoma;background:#0a0a0f;color:#eee;padding:20px;text-align:center}";
    h += "h1{color:#00d4ff}input,button{padding:12px;margin:8px;border-radius:8px;border:none;font-size:1rem}";
    h += "button{background:#00d4ff;color:#000;cursor:pointer;font-weight:bold}</style></head><body>";
    h += "<h1>HIZMOS OTA Update</h1>";
    h += "<form method='POST' action='/update' enctype='multipart/form-data'>";
    h += "<input type='file' name='firmware' accept='.bin' required><br>";
    h += "<button type='submit'>Upload Firmware</button></form></body></html>";
    server.send(200,"text/html", h);
  });
  server.on("/update", HTTP_POST, [](){
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    delay(1000); ESP.restart();
  }, [](){
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) Serial.printf("OTA OK: %u bytes\n", upload.totalSize);
      else Update.printError(Serial);
    }
  });

  server.begin();
  Serial.println("=== Ready ===");
}

void loop() {
  server.handleClient();
  webSocket.loop();
  rawProcessCapture();
  processSigGen();
  if (WiFi.status() == WL_CONNECTED || WiFi.softAPgetStationNum() > 0) {
    mqttReconnect();
    mqttClient.loop();
  }
  delay(1);
}
