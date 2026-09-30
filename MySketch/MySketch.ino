// ============================================================
// HIZMOS v9.0 - Smart Remote + RF Audio Receiver
// ESP32 + SI4432 + OTA + WiFi
// ============================================================
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RadioLib.h>
#include <LittleFS.h>
#include <Update.h>

// ========== پین‌های SI4432 ==========
#define SI4432_CS       27
#define SI4432_IRQ      35
#define SI4432_SDN      32
#define SI4432_RX_DATA  34
#define HSPI_SCK        14
#define HSPI_MISO       33
#define HSPI_MOSI       13

Si4432 radio_si = new Module(SI4432_CS, SI4432_IRQ, SI4432_SDN);
WebServer server(80);

bool si_ok = false;
float si_freq = 433.92;

// ========== بافر ضبط هوشمند ==========
#define RAW_MAX 1024
uint16_t rawPulses[RAW_MAX];
int rawCount = 0;
bool capturing = false;
uint32_t capStart = 0;
uint32_t lastTime = 0;
bool lastState = false;
uint32_t capDuration = 3000; // 3 ثانیه فرصت برای پیدا کردن فرکانس

// ========== صفحه HTML بهینه ==========
const char HTML_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="fa" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HIZMOS v9.0</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Tahoma,Arial;background:#0a0a0f;color:#e0e0e0;padding:8px;font-size:14px}
.box{max-width:520px;margin:0 auto;background:#12121a;border-radius:14px;padding:12px;border:1px solid #1a1a2e}
h1{color:#00d4ff;font-size:1rem;margin-bottom:8px;text-align:center}
h3{color:#00d4ff;font-size:0.85rem;margin:10px 0 4px}
.status{padding:8px;border-radius:6px;margin:5px 0;font-size:0.85rem;text-align:center}
.ok{background:#0a1f0a;border:1px solid #0a5;color:#8f8}
.na{background:#1f0a0a;border:1px solid #a00;color:#f88}
.btn{display:block;width:100%;padding:10px;margin:4px 0;border:none;border-radius:6px;background:#1a1a2e;color:#fff;font-size:0.85rem;cursor:pointer;text-align:right}
.btn.blue{background:#06c}.btn.green{background:#0a5}.btn.red{background:#a00}.btn.warn{background:#a80}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:4px}
.log{background:#000;border:1px solid #1a1a2e;border-radius:5px;padding:7px;font-family:monospace;font-size:0.65rem;color:#8f8;max-height:180px;overflow-y:auto;margin-top:4px;line-height:1.4;direction:ltr;text-align:left;white-space:pre-wrap}
input,select{background:#1a1a2e;color:#fff;border:1px solid #2a2a4e;border-radius:5px;padding:7px;font-size:0.8rem;width:100%;margin:3px 0}
label{font-size:0.72rem;color:#aaa;display:block;margin-top:4px}
.note{background:#0a1a0a;border:1px solid #0a5;border-radius:5px;padding:6px;font-size:0.68rem;color:#8f8;margin:4px 0;line-height:1.4}
</style>
</head>
<body>
<div class="box">
<h1>📡 HIZMOS v9.0</h1>
<div class="status" id="si">...</div>

<h3>📡 شکار ریموت (هوشمند)</h3>
<div class="note">دستگاه ۳ ثانیه گوش می‌دهد. دکمه ریموت را فشار دهید.</div>
<label>فرکانس (MHz):</label>
<input type="number" id="f" value="433.92" step="0.01">
<button class="btn green" onclick="hunt()">🎯 شکار و ضبط</button>
<button class="btn blue" onclick="play()">🔊 پخش آخرین ضبط</button>
<button class="btn green" onclick="replay()">📡 بازپخش با RF</button>
<div class="grid2">
<button class="btn blue" onclick="save()">💾 ذخیره</button>
<button class="btn blue" onclick="load()">📂 بارگذاری</button>
</div>
<button class="btn red" onclick="clearAll()">🗑 پاک کردن</button>

<h3>🎧 رادیو صوتی (Live Audio)</h3>
<label>فرکانس (MHz):</label>
<input type="number" id="af" value="433.92" step="0.01">
<button class="btn green" id="ab" onclick="toggleAudio()">▶️ شروع شنیدن</button>
<button class="btn blue" onclick="singleAudio()">🔊 شنیدن یکباره</button>

<h3>📊 ابزارها</h3>
<button class="btn blue" onclick="go('/spectrum')">📊 اسکن طیف</button>
<button class="btn blue" onclick="go('/rssi')">📶 تست RSSI</button>

<h3>📦 OTA Update</h3>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin" required>
<button class="btn green" type="submit">📤 آپلود فریمور</button>
</form>

<h3>ℹ️ سیستم</h3>
<button class="btn blue" onclick="go('/sys')">اطلاعات سیستم</button>
<button class="btn warn" onclick="if(confirm('ری‌استارت؟'))location='/reboot'">🔄 ری‌استارت</button>

<h3>خروجی</h3>
<div class="log" id="log">آماده...</div>
</div>

<script>
var ac=null, audioOn=false, audioTimer=null, lastPulses=null;

function ia(){if(!ac){try{ac=new(window.AudioContext||window.webkitAudioContext)();}catch(e){return null;}}if(ac.state==='suspended')ac.resume();return ac;}

function pp(p,f){var c=ia();if(!c||!p||!p.length)return 0;f=f||2500;var sr=c.sampleRate,tu=0;for(var i=0;i<p.length;i++)tu+=p[i];var ts=tu/1e6;if(ts<0.001)return 0;var ns=Math.ceil(ts*sr),b=c.createBuffer(1,ns,sr),d=b.getChannelData(0),o=0,s=true,ph=0,dp=2*Math.PI*f/sr;for(var i=0;i<p.length;i++){var du=Math.round(p[i]*sr/1e6);if(s){for(var j=0;j<du&&o+j<ns;j++){d[o+j]=Math.sin(ph)*0.4;ph+=dp;if(ph>2*Math.PI)ph-=2*Math.PI;}}o+=du;s=!s;}var src=c.createBufferSource();src.buffer=b;src.connect(c.destination);src.start();return ts;}

function lg(s){var a=document.getElementById('log');a.innerText=s;a.scrollTop=a.scrollHeight;}
function go(u){lg('...');fetch(u).then(r=>r.text()).then(t=>lg(t)).catch(e=>lg('Err: '+e));}
function sl(m){return new Promise(r=>setTimeout(r,m));}

function rs(){fetch('/si_status').then(r=>r.json()).then(d=>{var e=document.getElementById('si');if(d.ok){e.className='status ok';e.innerText='✅ SI4432 OK';}else{e.className='status na';e.innerText='❌ SI4432 N/A';}}).catch(()=>{});}

function hunt(){
  ia();var f=document.getElementById('f').value;
  fetch('/set_freq?freq='+f).then(()=>{
    fetch('/capture').then(()=>{
      lg('🎯 Hunting for 3s... Press remote NOW!');
      setTimeout(()=>{
        fetch('/pulses').then(r=>r.json()).then(d=>{
          if(d.count>0){
            lastPulses=d.pulses;
            var du=pp(d.pulses,2000);
            lg('✅ Found '+d.count+' valid pulses! Playing...');
          } else { lg('❌ No signal found. Try again.'); }
        });
      },3200);
    });
  });
}

function play(){ia();if(!lastPulses){lg('No previous capture');return;}pp(lastPulses,2000);lg('▶ Played last capture');}
function replay(){go('/replay');}
function save(){go('/save');}
function load(){go('/load');}
function clearAll(){lastPulses=null;go('/clear');}

function toggleAudio(){if(audioOn){stopAudio();}else{startAudio();}}

function startAudio(){
  ia();var f=document.getElementById('af').value;
  fetch('/set_freq?freq='+f).then(()=>{
    audioOn=true;
    var b=document.getElementById('ab');
    b.innerText='⏹ توقف';b.className='btn red';
    lg('🎧 Live audio...');
    audioLoop();
  });
}

function stopAudio(){
  audioOn=false;
  if(audioTimer){clearTimeout(audioTimer);audioTimer=null;}
  var b=document.getElementById('ab');
  if(b){b.innerText='▶️ شروع شنیدن';b.className='btn green';}
}

async function audioLoop(){
  while(audioOn){
    try{
      await fetch('/audio_capture');
      await sl(500); // بافر کوچک‌تر برای صدا
      var r=await fetch('/pulses');
      var d=await r.json();
      if(d.pulses && d.pulses.length>0){
        pp(d.pulses, 3000); // فرکانس صوتی بالاتر
        await sl(300);
      } else { await sl(100); }
    }catch(e){ await sl(200); }
  }
}

function singleAudio(){
  ia();var f=document.getElementById('af').value;
  fetch('/set_freq?freq='+f).then(()=>{
    fetch('/audio_capture').then(()=>{
      lg('🎙️ Recording 0.5s audio...');
      setTimeout(()=>{
        fetch('/pulses').then(r=>r.json()).then(d=>{
          if(d.pulses && d.pulses.length>0){ pp(d.pulses,3000); lg('▶ Played audio'); }
          else lg('No signal');
        });
      },700);
    });
  });
}

rs();setInterval(rs,3000);
</script>
</body>
</html>
)HTML";

// ========== توابع کمکی ==========
void initSI(){
  pinMode(SI4432_SDN, OUTPUT);
  digitalWrite(SI4432_SDN, LOW);
  delay(50);
  SPI.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, SI4432_CS);
  delay(30);
  int st = radio_si.begin(si_freq);
  if(st == RADIOLIB_ERR_NONE){
    si_ok = true;
    radio_si.setOutputPower(20);
    radio_si.setBitRate(4.8);
    radio_si.startReceive();
  }
}

// ========== ضبط هوشمند با فیلتر نویز ==========
void startCapture(uint32_t dur){
  rawCount = 0;
  capturing = true;
  capStart = millis();
  capDuration = dur;
  lastState = digitalRead(SI4432_RX_DATA);
  lastTime = micros();
}

void processCapture(){
  if(!capturing) return;
  if(millis() - capStart > capDuration){ capturing = false; return; }
  bool cur = digitalRead(SI4432_RX_DATA);
  if(cur != lastState){
    uint32_t now = micros();
    uint32_t d = now - lastTime;
    // ⚡ فیلتر نویز: فقط پالس‌های معتبر
    if(d > 100 && rawCount < RAW_MAX){
      rawPulses[rawCount++] = (uint16_t)min(d, (uint32_t)65535);
      lastTime = now;
      lastState = cur;
    }
  }
}

void replayRF(){
  if(rawCount == 0 || !si_ok) return;
  radio_si.setFrequency(si_freq);
  radio_si.setBitRate(4.8);
  radio_si.transmitDirect();
  bool state = true;
  for(int i = 0; i < rawCount; i++){
    digitalWrite(SI4432_SDN, state ? HIGH : LOW);
    delayMicroseconds(rawPulses[i]);
    state = !state;
  }
  digitalWrite(SI4432_SDN, LOW);
  radio_si.standby();
}

// ========== Setup ==========
void setup(){
  Serial.begin(115200);
  delay(300);
  LittleFS.begin(true);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("HIZMOS-AP", "hizmos123");
  initSI();

  server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html", HTML_PAGE); });
  server.on("/si_status", HTTP_GET, [](){ server.send(200, "application/json", si_ok ? F("{\"ok\":true}") : F("{\"ok\":false}")); });

  server.on("/set_freq", HTTP_GET, [](){
    if(server.hasArg("freq")){ si_freq = server.arg("freq").toFloat(); if(si_ok) radio_si.setFrequency(si_freq); }
    server.send(200, "text/plain", "OK");
  });

  server.on("/capture", HTTP_GET, [](){
    if(!si_ok) return server.send(503, "text/plain", "N/A");
    radio_si.setFrequency(si_freq);
    radio_si.setBitRate(4.8);
    radio_si.receiveDirect();
    startCapture(3000);
    server.send(200, "text/plain", "Recording");
  });

  server.on("/audio_capture", HTTP_GET, [](){
    if(!si_ok) return server.send(503, "text/plain", "N/A");
    radio_si.setFrequency(si_freq);
    radio_si.setBitRate(8.0); // نرخ بالاتر برای صدا
    radio_si.receiveDirect();
    startCapture(500);
    server.send(200, "text/plain", "Recording");
  });

  server.on("/pulses", HTTP_GET, [](){
    String j = "{\"count\":" + String(rawCount) + ",\"pulses\":[";
    for(int i = 0; i < rawCount; i++){ if(i>0) j += ","; j += String(rawPulses[i]); }
    j += "]}";
    server.send(200, "application/json", j);
  });

  server.on("/replay", HTTP_GET, [](){ replayRF(); server.send(200, "text/plain", "Replayed"); });
  server.on("/save", HTTP_GET, [](){ /* کد ذخیره‌سازی */ });
  server.on("/load", HTTP_GET, [](){ /* کد بارگذاری */ });
  server.on("/clear", HTTP_GET, [](){ rawCount = 0; server.send(200, "text/plain", "Cleared"); });
  server.on("/spectrum", HTTP_GET, [](){ /* کد اسکن طیف */ });
  server.on("/rssi", HTTP_GET, [](){ /* کد تست RSSI */ });
  server.on("/sys", HTTP_GET, [](){ server.send(200, "text/plain", "System OK"); });
  server.on("/reboot", HTTP_GET, [](){ server.send(200, "text/plain", "OK"); delay(500); ESP.restart(); });

  server.on("/update", HTTP_POST, [](){
    server.send(200, "text/plain", Update.hasError() ? "FAIL" : "OK");
    delay(1000); ESP.restart();
  }, [](){
    HTTPUpload& up = server.upload();
    if(up.status == UPLOAD_FILE_START) Update.begin(UPDATE_SIZE_UNKNOWN);
    else if(up.status == UPLOAD_FILE_WRITE) Update.write(up.buf, up.currentSize);
    else if(up.status == UPLOAD_FILE_END) Update.end(true);
  });

  server.begin();
}

void loop(){
  server.handleClient();
  processCapture();
}
