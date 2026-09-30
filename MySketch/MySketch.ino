// ============================================================
// HIZMOS v7.1 - Remote Capture + Browser Audio + OTA
// ============================================================
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RadioLib.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Update.h>

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
int si_error = 0;

// بافر ضبط
#define RAW_MAX 1024
uint16_t rawPulses[RAW_MAX];
int rawCount = 0;
bool capturing = false;
uint32_t capStart = 0;
uint32_t lastTime = 0;
bool lastState = false;
uint32_t capDuration = 2000;

// Signal Gen
bool genActive = false;
String genPattern = "10101010";
int genIdx = 0;

const char HTML_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="fa" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HIZMOS v7.1</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Tahoma,Arial;background:#0a0a0f;color:#e0e0e0;padding:10px;font-size:14px}
.box{max-width:520px;margin:0 auto;background:#12121a;border-radius:14px;padding:14px;border:1px solid #1a1a2e}
h1{color:#00d4ff;font-size:1.1rem;margin-bottom:10px;text-align:center}
h3{color:#00d4ff;font-size:0.9rem;margin:12px 0 6px}
.status{padding:10px;border-radius:8px;margin:6px 0;font-size:0.9rem;text-align:center}
.ok{background:#0a1f0a;border:1px solid #0a5;color:#8f8}
.na{background:#1f0a0a;border:1px solid #a00;color:#f88}
.btn{display:block;width:100%;padding:11px;margin:5px 0;border:none;border-radius:8px;background:#1a1a2e;color:#fff;font-size:0.9rem;cursor:pointer;text-align:right}
.btn:hover{background:#2a2a4e}
.btn.blue{background:#06c}
.btn.green{background:#0a5}
.btn.red{background:#a00}
.btn.warn{background:#a80}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:5px}
.log{background:#000;border:1px solid #1a1a2e;border-radius:6px;padding:9px;font-family:monospace;font-size:0.7rem;color:#8f8;max-height:200px;overflow-y:auto;margin-top:6px;line-height:1.4;direction:ltr;text-align:left;white-space:pre-wrap}
input,select{background:#1a1a2e;color:#fff;border:1px solid #2a2a4e;border-radius:6px;padding:8px;font-size:0.85rem;width:100%;margin:4px 0}
label{font-size:0.78rem;color:#aaa;display:block;margin-top:5px}
.note{background:#0a1a0a;border:1px solid #0a5;border-radius:6px;padding:8px;font-size:0.72rem;color:#8f8;margin:5px 0;line-height:1.5}
.playing{background:#0a5 !important;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.6}}
</style>
</head>
<body>
<div class="box">
<h1>📡 HIZMOS v7.1</h1>
<div class="status" id="siStatus">...</div>

<h3>📻 SI4432</h3>
<button class="btn blue" onclick="go('/diag')">🔍 تست کامل</button>
<button class="btn blue" onclick="go('/spectrum')">📊 اسکن طیف ۴۳۰-۴۴۰</button>
<button class="btn blue" onclick="go('/rssi')">📶 RSSI زنده</button>

<h3>📡 کپی ریموت ۴۳۳MHz</h3>
<div class="note">دکمه ریموت را پس از شروع ضبط، فشار بده. فایل صوتی از اسپیکر گوشی پخش می‌شود.</div>
<label>فرکانس (MHz):</label>
<input type="number" id="freq" value="433.92" step="0.01">
<button class="btn green" onclick="capturePlay()">🔴 ضبط + پخش با گوشی</button>
<button class="btn blue" onclick="captureOnly()">🎙️ فقط ضبط ۲ ثانیه</button>
<button class="btn blue" onclick="playLast()">🔊 پخش آخرین ضبط</button>
<button class="btn green" onclick="replayRF()">📡 بازپخش با RF</button>
<div class="grid2">
<button class="btn blue" onclick="go('/save')">💾 ذخیره</button>
<button class="btn blue" onclick="go('/load')">📂 بارگذاری</button>
</div>
<button class="btn red" onclick="go('/clear')">🗑 پاک کردن</button>

<h3>🎧 شنود صوتی زنده</h3>
<label>فرکانس (MHz):</label>
<input type="number" id="listenFreq" value="433.92" step="0.01">
<button class="btn green" id="btnListen" onclick="toggleListen()">▶️ شروع شنیدن</button>

<h3>📡 مولد سیگنال</h3>
<label>فرکانس (MHz):</label>
<input type="number" id="genFreq" value="433.92" step="0.01">
<label>الگو (0 و 1):</label>
<input type="text" id="genPat" value="1010101010101010">
<button class="btn green" onclick="startGen()">شروع</button>
<button class="btn red" onclick="go('/gen_stop')">توقف</button>

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
var audioCtx=null, listening=false, listenTimer=null, lastPulses=null;

function initAudio(){
  if(!audioCtx){try{audioCtx=new(window.AudioContext||window.webkitAudioContext)();}catch(e){return null;}}
  if(audioCtx.state==='suspended')audioCtx.resume();
  return audioCtx;
}

function playPulses(pulses,freqHz){
  var ctx=initAudio();
  if(!ctx||!pulses||pulses.length===0)return 0;
  freqHz=freqHz||2500;
  var sr=ctx.sampleRate, totalUs=0;
  for(var i=0;i<pulses.length;i++)totalUs+=pulses[i];
  var totalSec=totalUs/1000000;
  if(totalSec<0.001)return 0;
  var samples=Math.ceil(totalSec*sr);
  var buf=ctx.createBuffer(1,samples,sr);
  var data=buf.getChannelData(0);
  var offset=0,state=true,phase=0,dphase=2*Math.PI*freqHz/sr;
  for(var i=0;i<pulses.length;i++){
    var dur=Math.round(pulses[i]*sr/1000000);
    if(state){for(var j=0;j<dur&&offset+j<samples;j++){data[offset+j]=Math.sin(phase)*0.4;phase+=dphase;if(phase>2*Math.PI)phase-=2*Math.PI;}}
    offset+=dur;state=!state;
  }
  var src=ctx.createBufferSource();
  src.buffer=buf;
  src.connect(ctx.destination);
  src.start();
  return totalSec;
}

function log(s){var a=document.getElementById('log');a.innerText=s;a.scrollTop=a.scrollHeight;}
function go(url){log('...');fetch(url).then(r=>r.text()).then(t=>log(t)).catch(e=>log('Error: '+e));}
function sleep(ms){return new Promise(r=>setTimeout(r,ms));}

function refreshStatus(){
  fetch('/si_status').then(r=>r.json()).then(d=>{
    var el=document.getElementById('siStatus');
    if(d.ok){el.className='status ok';el.innerText='✅ SI4432 OK ('+d.msg+')';}
    else{el.className='status na';el.innerText='❌ SI4432 N/A — '+d.msg;}
  }).catch(()=>{});
}

function setFreq(f){
  return fetch('/set_freq?freq='+f).then(r=>r.text());
}

function capturePlay(){
  initAudio();
  var f=document.getElementById('freq').value;
  setFreq(f).then(()=>{
    fetch('/capture').then(()=>{
      log('🎙️ Recording 2s... Press remote NOW!');
      setTimeout(()=>{
        fetch('/pulses').then(r=>r.json()).then(d=>{
          if(d.pulses&&d.pulses.length>0){
            lastPulses=d.pulses;
            var dur=playPulses(d.pulses,2000);
            log('▶ Captured '+d.pulses.length+' pulses, played '+dur.toFixed(2)+'s');
          } else log('No signal captured');
        });
      },2200);
    });
  });
}

function captureOnly(){
  var f=document.getElementById('freq').value;
  setFreq(f).then(()=>{
    fetch('/capture').then(()=>{
      log('🎙️ Recording 2s...');
      setTimeout(()=>{
        fetch('/pulses').then(r=>r.json()).then(d=>{
          lastPulses=d.pulses;
          log('Captured '+(d.pulses?d.pulses.length:0)+' pulses');
        });
      },2200);
    });
  });
}

function playLast(){
  initAudio();
  if(!lastPulses){log('No previous capture');return;}
  playPulses(lastPulses,2000);
  log('▶ Played last capture');
}

function replayRF(){go('/replay');}

function toggleListen(){
  if(listening){stopListen();}
  else{startListen();}
}

function startListen(){
  initAudio();
  var f=document.getElementById('listenFreq').value;
  setFreq(f).then(()=>{
    listening=true;
    var btn=document.getElementById('btnListen');
    btn.innerText='⏹ توقف';
    btn.className='btn red playing';
    log('🎧 Listening...');
    listenLoop();
  });
}

function stopListen(){
  listening=false;
  if(listenTimer){clearTimeout(listenTimer);listenTimer=null;}
  var btn=document.getElementById('btnListen');
  if(btn){btn.innerText='▶️ شروع شنیدن';btn.className='btn green';}
}

async function listenLoop(){
  while(listening){
    try{
      await fetch('/capture');
      await sleep(2200);
      var r=await fetch('/pulses');
      var d=await r.json();
      if(d.pulses&&d.pulses.length>0){
        var dur=playPulses(d.pulses,2500);
        log('▶ '+d.pulses.length+' pulses ('+dur.toFixed(2)+'s)');
        await sleep(dur*1000);
      } else await sleep(300);
    }catch(e){log('Err: '+e);await sleep(500);}
  }
}

function startGen(){
  var f=document.getElementById('genFreq').value;
  var p=document.getElementById('genPat').value;
  go('/gen_start?freq='+f+'&pattern='+p);
}

refreshStatus();
setInterval(refreshStatus,3000);
</script>
</body>
</html>
)HTML";

// ========== SI4432 ==========
void initSI(){
  Serial.println("Init SI4432...");
  pinMode(SI4432_SDN, OUTPUT);
  digitalWrite(SI4432_SDN, LOW);
  delay(100);
  SPI.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, SI4432_CS);
  delay(50);
  int st = radio_si.begin(si_freq);
  if(st == RADIOLIB_ERR_NONE){
    si_ok = true;
    Serial.println("SI4432 OK");
    radio_si.setOutputPower(10);
    radio_si.setBitRate(2.4);
    radio_si.startReceive();
  } else {
    si_ok = false;
    si_error = st;
    Serial.printf("SI4432 FAIL code=%d\n", st);
  }
}

// ========== Capture ==========
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
  if(millis() - capStart > capDuration){
    capturing = false;
    return;
  }
  bool cur = digitalRead(SI4432_RX_DATA);
  if(cur != lastState){
    uint32_t now = micros();
    uint32_t d = now - lastTime;
    if(d > 30 && rawCount < RAW_MAX){
      rawPulses[rawCount++] = (uint16_t)min(d, (uint32_t)65535);
      lastTime = now;
      lastState = cur;
    }
  }
}

void replayRF(){
  if(rawCount == 0 || !si_ok) return;
  radio_si.setFrequency(si_freq);
  radio_si.setBitRate(2.4);
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

void processGen(){
  if(!genActive || genPattern.length() == 0) return;
  char c = genPattern[genIdx];
  digitalWrite(SI4432_SDN, c == '1' ? HIGH : LOW);
  delayMicroseconds(500);
  genIdx = (genIdx + 1) % genPattern.length();
}

// ========== Setup ==========
void setup(){
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HIZMOS v7.1 ===");

  pinMode(SI4432_RX_DATA, INPUT);

  if(!LittleFS.begin(true)) Serial.println("LittleFS fail");

  initSI();

  WiFi.softAP("HIZMOS-AP", "hizmos123");
  Serial.print("AP: "); Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html", HTML_PAGE); });

  server.on("/si_status", HTTP_GET, [](){
    String j = "{\"ok\":" + String(si_ok ? "true" : "false");
    j += ",\"msg\":\"" + String(si_ok ? "Init OK" : ("code " + String(si_error))) + "\"}";
    server.send(200, "application/json", j);
  });

  server.on("/diag", HTTP_GET, [](){
    String r = "=== SI4432 Diagnostic ===\n";
    r += "Pins: CS=" + String(SI4432_CS) + " IRQ=" + String(SI4432_IRQ) + " SDN=" + String(SI4432_SDN) + "\n";
    r += "SCK=" + String(HSPI_SCK) + " MISO=" + String(HSPI_MISO) + " MOSI=" + String(HSPI_MOSI) + "\n\n";
    r += "SDN pin: " + String(digitalRead(SI4432_SDN)) + " (want 0)\n";
    r += "IRQ pin: " + String(digitalRead(SI4432_IRQ)) + " (want 1)\n\n";
    r += "begin(): " + String(si_ok ? "OK" : ("FAIL code " + String(si_error))) + "\n";
    if(si_ok){
      r += "Chip ver: " + String(radio_si.getChipVersion()) + "\n";
      radio_si.startReceive();
      delay(50);
      r += "RSSI: " + String(radio_si.getRSSI(), 2) + " dBm\n";
    }
    server.send(200, "text/plain", r);
  });

  server.on("/spectrum", HTTP_GET, [](){
    if(!si_ok){ server.send(200, "text/plain", "SI4432 N/A"); return; }
    String r = "=== Spectrum 430-440 MHz ===\n";
    for(int f = 430; f <= 440; f++){
      radio_si.setFrequency(f);
      radio_si.startReceive();
      delay(50);
      r += String(f) + " MHz : " + String(radio_si.getRSSI(), 2) + " dBm\n";
    }
    radio_si.setFrequency(si_freq);
    radio_si.startReceive();
    server.send(200, "text/plain", r);
  });

  server.on("/rssi", HTTP_GET, [](){
    if(!si_ok){ server.send(200, "text/plain", "SI4432 N/A"); return; }
    radio_si.startReceive();
    delay(50);
    String r = "RSSI @ " + String(si_freq, 2) + " MHz : " + String(radio_si.getRSSI(), 2) + " dBm\n";
    server.send(200, "text/plain", r);
  });

  server.on("/set_freq", HTTP_GET, [](){
    if(server.hasArg("freq")){
      si_freq = server.arg("freq").toFloat();
      if(si_ok){ radio_si.setFrequency(si_freq); radio_si.startReceive(); }
      server.send(200, "text/plain", "Freq: " + String(si_freq, 2));
    } else server.send(400, "text/plain", "missing");
  });

  server.on("/capture", HTTP_GET, [](){
    if(!si_ok){ server.send(503, "text/plain", "N/A"); return; }
    radio_si.setFrequency(si_freq);
    radio_si.setBitRate(2.4);
    radio_si.receiveDirect();
    startCapture(2000);
    server.send(200, "text/plain", "Recording 2s...");
  });

  server.on("/pulses", HTTP_GET, [](){
    String j = "{\"count\":" + String(rawCount) + ",\"freq\":" + String(si_freq, 2) + ",\"pulses\":[";
    for(int i = 0; i < rawCount; i++){
      if(i > 0) j += ",";
      j += String(rawPulses[i]);
    }
    j += "]}";
    server.send(200, "application/json", j);
  });

  server.on("/replay", HTTP_GET, [](){
    if(!si_ok){ server.send(503, "text/plain", "N/A"); return; }
    if(rawCount == 0){ server.send(400, "text/plain", "no signal"); return; }
    replayRF();
    server.send(200, "text/plain", "Replayed " + String(rawCount) + " pulses via RF\n");
  });

  server.on("/save", HTTP_GET, [](){
    if(rawCount == 0){ server.send(400, "text/plain", "no signal"); return; }
    StaticJsonDocument<4096> doc;
    doc["freq"] = si_freq;
    doc["count"] = rawCount;
    JsonArray arr = doc.createNestedArray("pulses");
    for(int i = 0; i < rawCount; i++) arr.add(rawPulses[i]);
    File f = LittleFS.open("/remote.json", "w");
    if(f){ serializeJson(doc, f); f.close(); server.send(200, "text/plain", "Saved\n"); }
    else server.send(500, "text/plain", "Fail");
  });

  server.on("/load", HTTP_GET, [](){
    if(!LittleFS.exists("/remote.json")){ server.send(404, "text/plain", "no file"); return; }
    File f = LittleFS.open("/remote.json", "r");
    String c = f.readString(); f.close();
    StaticJsonDocument<4096> doc;
    if(deserializeJson(doc, c)){ server.send(400, "text/plain", "bad json"); return; }
    si_freq = doc["freq"] | 433.92;
    rawCount = 0;
    for(JsonVariant v : doc["pulses"].as<JsonArray>()){
      if(rawCount >= RAW_MAX) break;
      rawPulses[rawCount++] = v.as<uint16_t>();
    }
    server.send(200, "text/plain", "Loaded " + String(rawCount) + " pulses\n");
  });

  server.on("/clear", HTTP_GET, [](){
    rawCount = 0;
    server.send(200, "text/plain", "Cleared\n");
  });

  server.on("/gen_start", HTTP_GET, [](){
    if(server.hasArg("freq") && server.hasArg("pattern")){
      float f = server.arg("freq").toFloat();
      genPattern = server.arg("pattern");
      genIdx = 0;
      genActive = true;
      if(si_ok){
        radio_si.setFrequency(f);
        radio_si.setBitRate(2.4);
        radio_si.transmitDirect();
      }
      server.send(200, "text/plain", "Gen started\n");
    } else server.send(400, "text/plain", "missing");
  });

  server.on("/gen_stop", HTTP_GET, [](){
    genActive = false;
    digitalWrite(SI4432_SDN, LOW);
    if(si_ok) radio_si.standby();
    server.send(200, "text/plain", "Gen stopped\n");
  });

  server.on("/sys", HTTP_GET, [](){
    String r = "=== System ===\n";
    r += "Chip: " + String(ESP.getChipModel()) + "\n";
    r += "CPU: " + String(ESP.getCpuFreqMHz()) + " MHz\n";
    r += "Free Heap: " + String(ESP.getFreeHeap()) + "\n";
    r += "Uptime: " + String(millis()/1000) + " s\n";
    server.send(200, "text/plain", r);
  });

  server.on("/reboot", HTTP_GET, [](){
    server.send(200, "text/plain", "Rebooting...");
    delay(500); ESP.restart();
  });

  server.on("/update", HTTP_POST, [](){
    server.send(200, "text/plain", Update.hasError() ? "FAIL" : "OK - Rebooting");
    delay(1000); ESP.restart();
  }, [](){
    HTTPUpload& up = server.upload();
    if(up.status == UPLOAD_FILE_START){
      if(!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if(up.status == UPLOAD_FILE_WRITE){
      if(Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
    } else if(up.status == UPLOAD_FILE_END){
      if(Update.end(true)) Serial.printf("OTA OK: %u\n", up.totalSize);
      else Update.printError(Serial);
    }
  });

  server.begin();
  Serial.println("=== Ready ===");
}

void loop(){
  server.handleClient();
  processCapture();
  processGen();
  delay(1);
}
