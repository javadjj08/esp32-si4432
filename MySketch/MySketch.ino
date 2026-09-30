/*
 * ============================================================
 *  ESP32 + SI4432 SubGHz RAW Recorder / Replayer (بهینه‌شده)
 *  حالت AP | پین D34 | سازگار با Flipper Zero .sub
 *  تشخیص خودکار + WebSocket UI + تنظیمات + OTA وب
 * ============================================================
 */

#include <WiFi.h>
#include <SPIFFS.h>
#include <RadioLib.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Update.h>

// ==================== تنظیمات AP ====================
const char* AP_SSID     = "SI4432-AP";
const char* AP_PASSWORD = "si4432admin";

// ==================== مقادیر پیش‌فرض پین‌ها ====================
#define DEF_PIN_CS      5
#define DEF_PIN_IRQ     2
#define DEF_PIN_SDN     4
#define DEF_PIN_GPIO2   34   // D34 (امن شده)
#define DEF_FREQ_MHZ    433.92
#define DEF_BITRATE     4.8
#define DEF_RX_BW       100.0
#define DEF_TX_POWER    10
#define DEF_RSSI_THR    0x20
#define DEF_MIN_PULSE   50
#define DEF_TIMEOUT_MS  8000

// ==================== ساختار تنظیمات ====================
struct Config {
  uint8_t pinCS;
  uint8_t pinIRQ;
  uint8_t pinSDN;
  uint8_t pinGPIO2;
  float freqMHz;
  float bitrate;
  float rxBW;
  int   txPower;
  uint8_t rssiThr;
  uint16_t minPulse;
  uint32_t timeoutMs;
} cfg;

Preferences prefs;

// ==================== اشیاء اصلی ====================
Si4432* radio = nullptr;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ==================== پارامترهای ضبط ====================
#define MAX_PULSES 2048
uint16_t pulseTimings[MAX_PULSES];
volatile uint16_t pulseCount = 0;
volatile bool capturing = false;
volatile bool autoMode = false;
volatile bool signalDetected = false;
volatile unsigned long lastEdgeUs = 0;
volatile int lastLevel = LOW;
unsigned long lastCaptureTime = 0;
String lastSignalName = "";

// ==================== حالت‌های سیستم ====================
enum Mode { MODE_IDLE, MODE_RX, MODE_TX };
Mode currentMode = MODE_IDLE;

// ==================== توابع تنظیمات ====================

void loadConfig() {
  prefs.begin("subghz", true);
  cfg.pinCS     = prefs.getUChar("pinCS", DEF_PIN_CS);
  cfg.pinIRQ    = prefs.getUChar("pinIRQ", DEF_PIN_IRQ);
  cfg.pinSDN    = prefs.getUChar("pinSDN", DEF_PIN_SDN);
  cfg.pinGPIO2  = prefs.getUChar("pinGPIO2", DEF_PIN_GPIO2);
  cfg.freqMHz   = prefs.getFloat("freq", DEF_FREQ_MHZ);
  cfg.bitrate   = prefs.getFloat("bitrate", DEF_BITRATE);
  cfg.rxBW      = prefs.getFloat("rxBW", DEF_RX_BW);
  cfg.txPower   = prefs.getInt("txPower", DEF_TX_POWER);
  cfg.rssiThr   = prefs.getUChar("rssiThr", DEF_RSSI_THR);
  cfg.minPulse  = prefs.getUShort("minPulse", DEF_MIN_PULSE);
  cfg.timeoutMs = prefs.getULong("timeout", DEF_TIMEOUT_MS);
  prefs.end();
  Serial.println("[CFG] بارگذاری شد");
}

void saveConfig() {
  prefs.begin("subghz", false);
  prefs.putUChar("pinCS", cfg.pinCS);
  prefs.putUChar("pinIRQ", cfg.pinIRQ);
  prefs.putUChar("pinSDN", cfg.pinSDN);
  prefs.putUChar("pinGPIO2", cfg.pinGPIO2);
  prefs.putFloat("freq", cfg.freqMHz);
  prefs.putFloat("bitrate", cfg.bitrate);
  prefs.putFloat("rxBW", cfg.rxBW);
  prefs.putInt("txPower", cfg.txPower);
  prefs.putUChar("rssiThr", cfg.rssiThr);
  prefs.putUShort("minPulse", cfg.minPulse);
  prefs.putULong("timeout", cfg.timeoutMs);
  prefs.end();
  Serial.println("[CFG] ذخیره شد");
}

// ==================== توابع وقفه ====================

void IRAM_ATTR gpio2PulseHandler() {
  if (!capturing) return;
  unsigned long now = micros();
  int level = digitalRead(cfg.pinGPIO2);
  if (level != lastLevel) {
    unsigned long duration = now - lastEdgeUs;
    if (duration >= cfg.minPulse && pulseCount < MAX_PULSES) {
      pulseTimings[pulseCount] = (uint16_t)(duration > 65535 ? 65535 : duration);
      pulseCount++;
    }
    lastEdgeUs = now;
    lastLevel = level;
  }
}

void IRAM_ATTR onSignalDetected() {
  signalDetected = true;
}

// ==================== ارتباط WebSocket ====================

void wsSendStatus(const char* mode, const char* statusText, uint16_t pulses) {
  StaticJsonDocument<256> doc;
  doc["type"] = "status";
  doc["mode"] = mode;
  doc["statusText"] = statusText;
  doc["pulses"] = pulses;
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

void wsSendConfig() {
  StaticJsonDocument<512> doc;
  doc["type"] = "config";
  doc["pinCS"] = cfg.pinCS;
  doc["pinIRQ"] = cfg.pinIRQ;
  doc["pinSDN"] = cfg.pinSDN;
  doc["pinGPIO2"] = cfg.pinGPIO2;
  doc["freq"] = cfg.freqMHz;
  doc["bitrate"] = cfg.bitrate;
  doc["rxBW"] = cfg.rxBW;
  doc["txPower"] = cfg.txPower;
  doc["rssiThr"] = cfg.rssiThr;
  doc["minPulse"] = cfg.minPulse;
  doc["timeout"] = cfg.timeoutMs;
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

void wsLog(const String& msg) {
  ws.textAll(msg);
}

// ==================== راه‌اندازی رادیو ====================

bool initRadio() {
  if (radio) { delete radio; radio = nullptr; }
  radio = new Si4432(new Module(cfg.pinCS, cfg.pinIRQ, cfg.pinSDN));

  int state = radio->begin(cfg.freqMHz, cfg.bitrate, 5.0, cfg.rxBW, cfg.txPower, 16);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[RADIO] begin خطا: %d\n", state);
    return false;
  }

  state = radio->setModulation(RADIOLIB_SI443X_MODULATION_OOK);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[RADIO] setModulation خطا: %d\n", state);
    return false;
  }

  // پیکربندی GPIO2 به عنوان خروجی داده خام
  radio->SPIsetRegValue(RADIOLIB_SI443X_REG_GPIO2_CONFIG, 0x14, 4, 0);

  // آستانه RSSI
  radio->SPIsetRegValue(RADIOLIB_SI443X_REG_RSSI_THRESHOLD, cfg.rssiThr);

  // فعال‌سازی وقفه RSSI
  uint8_t intEnable2;
  radio->SPIreadRegister(RADIOLIB_SI443X_REG_INT_ENABLE_2, &intEnable2);
  intEnable2 |= 0x10;
  radio->SPIwriteRegister(RADIOLIB_SI443X_REG_INT_ENABLE_2, intEnable2);

  Serial.println("[RADIO] SI4432 آماده شد");
  return true;
}

// ==================== تغییر حالت ====================

void enterRxMode() {
  if (!radio) return;
  radio->standby();
  int state = radio->startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[RADIO] startReceive خطا: %d\n", state);
    return;
  }
  currentMode = MODE_RX;
  Serial.println("[RADIO] حالت RX فعال");
}

void enterTxMode() {
  if (!radio) return;
  radio->standby();
  currentMode = MODE_TX;
}

// ==================== ضبط و ذخیره ====================

void startCapture() {
  if (capturing || !radio) return;
  enterRxMode();
  delay(10);

  pulseCount = 0;
  lastEdgeUs = micros();
  lastLevel = digitalRead(cfg.pinGPIO2);
  lastCaptureTime = millis();

  attachInterrupt(digitalPinToInterrupt(cfg.pinGPIO2), gpio2PulseHandler, CHANGE);
  capturing = true;

  Serial.println("[CAPTURE] شروع شد");
  wsSendStatus("recording", "در حال ضبط...", 0);
}

String stopCapture() {
  if (!capturing) return "در حال ضبط نیست";
  detachInterrupt(digitalPinToInterrupt(cfg.pinGPIO2));
  capturing = false;

  Serial.printf("[CAPTURE] متوقف شد، %u پالس\n", pulseCount);

  if (pulseCount < 4) {
    wsSendStatus("idle", "سیگنال معتبری ضبط نشد", 0);
    return "پالس کم";
  }

  char filename[48];
  snprintf(filename, sizeof(filename), "/sig_%lu.sub", (unsigned long)(millis() / 1000));
  lastSignalName = String(filename);

  bool ok = saveSubFile(filename);
  if (!ok) {
    wsSendStatus("idle", "خطا در ذخیره‌سازی", 0);
    return "خطای نوشتن SPIFFS";
  }

  wsSendStatus("idle", "ضبط و ذخیره کامل شد", pulseCount);
  return "OK";
}

// ==================== خواندن/نوشتن فایل .sub ====================

bool saveSubFile(const char* path) {
  File f = SPIFFS.open(path, FILE_WRITE);
  if (!f) return false;

  f.println("Filetype: Flipper SubGhz RAW File");
  f.println("Version: 1");
  f.printf("Frequency: %lu\n", (unsigned long)(cfg.freqMHz * 1000000));
  f.println("Preset: FuriHalSubGhzPresetOok650Async");
  f.println("Protocol: RAW");
  f.print("RAW_Data: ");

  bool positive = true;
  int lineCount = 0;

  for (uint16_t i = 0; i < pulseCount; i++) {
    int32_t val = pulseTimings[i];
    if (val < cfg.minPulse) val = cfg.minPulse;
    if (positive) f.print(val); else f.print(-val);
    positive = !positive;

    lineCount++;
    if (lineCount >= 512) {
      f.println();
      f.print("RAW_Data: ");
      lineCount = 0;
    } else if (i < pulseCount - 1) {
      f.print(' ');
    }
  }
  f.println();
  f.close();

  Serial.printf("[SPIFFS] ذخیره شد: %s (%u پالس)\n", path, pulseCount);
  return true;
}

bool replaySubFile(const char* path) {
  if (!radio) return false;
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) {
    Serial.printf("[REPLAY] فایل یافت نشد: %s\n", path);
    wsLog("فایل یافت نشد!");
    return false;
  }

  uint16_t timings[MAX_PULSES];
  uint16_t count = 0;

  while (f.available() && count < MAX_PULSES) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.startsWith("RAW_Data:")) continue;

    int pos = 8;
    while (pos < (int)line.length() && count < MAX_PULSES) {
      while (pos < (int)line.length() && line[pos] == ' ') pos++;
      if (pos >= (int)line.length()) break;

      if (line[pos] == '-') pos++;
      else if (line[pos] == '+') pos++;

      int32_t val = 0;
      while (pos < (int)line.length() && isdigit(line[pos])) {
        val = val * 10 + (line[pos] - '0');
        pos++;
      }
      if (val == 0) continue;
      timings[count++] = (uint16_t)(val > 65535 ? 65535 : val);
    }
  }
  f.close();

  if (count < 4) {
    wsLog("داده معتبری برای بازپخش یافت نشد.");
    return false;
  }

  Serial.printf("[REPLAY] ارسال %u پالس از %s\n", count, path);
  wsLog("در حال بازپخش...");

  enterTxMode();
  delay(5);

  int state = radio->transmitDirect();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[REPLAY] transmitDirect خطا: %d\n", state);
    wsLog("خطا در شروع ارسال!");
    enterRxMode();
    return false;
  }

  for (uint16_t i = 0; i < count; i++) {
    uint32_t dur = timings[i];
    if (i % 2 == 0) {
      delayMicroseconds(dur);
    } else {
      radio->standby();
      delayMicroseconds(dur);
      radio->transmitDirect();
    }
  }

  radio->standby();
  enterRxMode();

  Serial.println("[REPLAY] تمام شد");
  wsLog("بازپخش کامل شد.");
  wsSendStatus("idle", "بازپخش کامل شد", pulseCount);
  return true;
}

// ==================== پردازش OTA وب ====================

void handleOTAUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) {
    Serial.printf("[OTA] شروع آپلود: %s\n", filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  }
  if (len) {
    if (Update.write(data, len) != len) {
      Update.printError(Serial);
    }
  }
  if (final) {
    if (Update.end(true)) {
      Serial.printf("[OTA] آپلود موفق: %u بایت\n", index + len);
    } else {
      Update.printError(Serial);
    }
  }
}

// ==================== رویدادهای WebSocket ====================

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] کلاینت #%u وصل شد\n", client->id());
    wsSendConfig();
    StaticJsonDocument<128> doc;
    doc["type"] = "status";
    doc["mode"] = capturing ? "recording" : "idle";
    doc["pulses"] = pulseCount;
    String out;
    serializeJson(doc, out);
    client->text(out);
  }
  else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] کلاینت #%u قطع شد\n", client->id());
  }
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      if (info->opcode == WS_TEXT) {
        data[len] = 0;
        StaticJsonDocument<512> doc;
        DeserializationError err = deserializeJson(doc, (char*)data);
        if (err) {
          Serial.printf("[WS] خطای JSON: %s\n", err.c_str());
          return;
        }

        const char* cmd = doc["cmd"];
        if (!cmd) return;

        if (strcmp(cmd, "start") == 0) {
          autoMode = false;
          detachInterrupt(digitalPinToInterrupt(cfg.pinIRQ));
          startCapture();
        }
        else if (strcmp(cmd, "stop") == 0) {
          stopCapture();
        }
        else if (strcmp(cmd, "replay") == 0) {
          if (SPIFFS.exists(lastSignalName.c_str())) {
            replaySubFile(lastSignalName.c_str());
          } else {
            File root = SPIFFS.open("/");
            File file = root.openNextFile();
            String latest = "";
            while (file) {
              if (!file.isDirectory() && String(file.name()).endsWith(".sub")) latest = String(file.name());
              file = root.openNextFile();
            }
            if (latest != "") replaySubFile(latest.c_str());
            else wsLog("هیچ فایلی برای بازپخش وجود ندارد.");
          }
        }
        else if (strcmp(cmd, "auto") == 0) {
          autoMode = !autoMode;
          if (autoMode) {
            enterRxMode();
            attachInterrupt(digitalPinToInterrupt(cfg.pinIRQ), onSignalDetected, FALLING);
            wsSendStatus("auto", "حالت شنود خودکار فعال", 0);
          } else {
            detachInterrupt(digitalPinToInterrupt(cfg.pinIRQ));
            wsSendStatus("idle", "حالت شنود خودکار غیرفعال", 0);
          }
        }
        else if (strcmp(cmd, "info") == 0) {
          StaticJsonDocument<256> d;
          d["type"] = "info";
          d["pulses"] = pulseCount;
          d["autoMode"] = autoMode;
          d["capturing"] = capturing;
          d["lastFile"] = lastSignalName;
          String out; serializeJson(d, out);
          ws.textAll(out);
        }
        else if (strcmp(cmd, "savecfg") == 0) {
          if (doc.containsKey("pinCS")) cfg.pinCS = doc["pinCS"];
          if (doc.containsKey("pinIRQ")) cfg.pinIRQ = doc["pinIRQ"];
          if (doc.containsKey("pinSDN")) cfg.pinSDN = doc["pinSDN"];
          if (doc.containsKey("pinGPIO2")) cfg.pinGPIO2 = doc["pinGPIO2"];
          if (doc.containsKey("freq")) cfg.freqMHz = doc["freq"];
          if (doc.containsKey("bitrate")) cfg.bitrate = doc["bitrate"];
          if (doc.containsKey("rxBW")) cfg.rxBW = doc["rxBW"];
          if (doc.containsKey("txPower")) cfg.txPower = doc["txPower"];
          if (doc.containsKey("rssiThr")) cfg.rssiThr = doc["rssiThr"];
          if (doc.containsKey("minPulse")) cfg.minPulse = doc["minPulse"];
          if (doc.containsKey("timeout")) cfg.timeoutMs = doc["timeout"];
          saveConfig();
          wsLog("تنظیمات ذخیره شد. برای اعمال، دستگاه را ری‌استارت کنید.");
          wsSendConfig();
        }
        else if (strcmp(cmd, "getcfg") == 0) {
          wsSendConfig();
        }
        else if (strcmp(cmd, "restart") == 0) {
          wsLog("دستگاه در حال ری‌استارت...");
          delay(500);
          ESP.restart();
        }
      }
    }
  }
}

// ==================== رابط HTML ====================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fa" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>کنترلر SubGHz - ESP32 + SI4432</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0;}
  body{font-family:Tahoma,sans-serif;background:#0d1117;color:#c9d1d9;text-align:center;padding:16px;}
  .container{max-width:520px;margin:auto;background:#161b22;padding:20px;border-radius:16px;box-shadow:0 0 30px rgba(0,0,0,.6);}
  h1{color:#58a6ff;font-size:1.3em;margin-bottom:4px;}
  .subtitle{color:#8b949e;font-size:.75em;margin-bottom:14px;}
  .status{background:#0d1117;border:1px solid #30363d;padding:10px;border-radius:10px;margin:10px 0;font-size:.85em;color:#7ee787;text-align:right;}
  .btn{display:block;width:100%;padding:12px;margin:8px 0;border:none;border-radius:10px;font-size:.95em;cursor:pointer;transition:all .2s;font-weight:bold;}
  .btn:active{transform:scale(.97);}
  .btn-record{background:#da3633;color:#fff;}
  .btn-stop{background:#d29922;color:#0d1117;}
  .btn-play{background:#238636;color:#fff;}
  .btn-info{background:#1f6feb;color:#fff;}
  .btn-auto{background:#8957e5;color:#fff;}
  .btn-auto.active{background:#da3633;animation:pulse 1.2s infinite;}
  @keyframes pulse{0%,100%{opacity:1;}50%{opacity:.6;}}
  .btn-settings{background:#6e7681;color:#fff;}
  .btn-ota{background:#2ea043;color:#fff;}
  .tabs{display:flex;gap:6px;margin:10px 0;}
  .tab{flex:1;padding:10px;background:#21262d;border-radius:8px;cursor:pointer;font-size:.85em;font-weight:bold;color:#8b949e;transition:.2s;}
  .tab.active{background:#1f6feb;color:#fff;}
  .panel{display:none;text-align:right;}
  .panel.active{display:block;}
  .field{margin:8px 0;}
  .field label{display:block;font-size:.8em;color:#8b949e;margin-bottom:3px;}
  .field input{width:100%;padding:8px;background:#0d1117;border:1px solid #30363d;border-radius:6px;color:#c9d1d9;font-size:.9em;}
  .field input:focus{outline:none;border-color:#1f6feb;}
  .row{display:flex;gap:8px;}
  .row .field{flex:1;}
  #log{background:#000;color:#7ee787;text-align:left;padding:10px;border-radius:8px;height:140px;overflow-y:auto;font-family:monospace;font-size:.72em;margin-top:12px;border:1px solid #30363d;direction:ltr;}
  .pulse-count{color:#f0883e;font-weight:bold;}
  .ota-note{font-size:.75em;color:#d29922;margin:6px 0;}
</style>
</head>
<body>
<div class="container">
  <h1>🎛️ کنترلر SubGHz</h1>
  <div class="subtitle">ESP32 + SI4432 | حالت AP | سازگار با Flipper Zero</div>

  <div class="status" id="status">وضعیت: آماده | پالس: <span class="pulse-count" id="pulseCount">0</span></div>

  <div class="tabs">
    <div class="tab active" onclick="switchTab('main')">🎮 کنترل</div>
    <div class="tab" onclick="switchTab('settings')">⚙️ تنظیمات</div>
    <div class="tab" onclick="switchTab('ota')">📡 OTA</div>
  </div>

  <div class="panel active" id="panel-main">
    <button class="btn btn-auto" id="btnAuto" onclick="sendCmd('auto')">🔍 حالت شنود خودکار</button>
    <button class="btn btn-record" onclick="sendCmd('start')">🔴 شروع ضبط دستی</button>
    <button class="btn btn-stop" onclick="sendCmd('stop')">⏹️ توقف و ذخیره</button>
    <button class="btn btn-play" onclick="sendCmd('replay')">▶️ بازپخش سیگنال</button>
    <button class="btn btn-info" onclick="sendCmd('info')">ℹ️ نمایش اطلاعات</button>
  </div>

  <div class="panel" id="panel-settings">
    <div class="row">
      <div class="field"><label>پایه CS</label><input type="number" id="pinCS" value="5"></div>
      <div class="field"><label>پایه IRQ</label><input type="number" id="pinIRQ" value="2"></div>
    </div>
    <div class="row">
      <div class="field"><label>پایه SDN</label><input type="number" id="pinSDN" value="4"></div>
      <div class="field"><label>پایه GPIO2</label><input type="number" id="pinGPIO2" value="34"></div>
    </div>
    <div class="row">
      <div class="field"><label>فرکانس (MHz)</label><input type="number" step="0.01" id="freq" value="433.92"></div>
      <div class="field"><label>نرخ بیت (kbps)</label><input type="number" step="0.1" id="bitrate" value="4.8"></div>
    </div>
    <div class="row">
      <div class="field"><label>پهنای باند RX (kHz)</label><input type="number" step="1" id="rxBW" value="100"></div>
      <div class="field"><label>توان ارسال (dBm)</label><input type="number" id="txPower" value="10"></div>
    </div>
    <div class="row">
      <div class="field"><label>آستانه RSSI (0-255)</label><input type="number" id="rssiThr" value="32"></div>
      <div class="field"><label>حداقل پالس (µs)</label><input type="number" id="minPulse" value="50"></div>
    </div>
    <div class="field"><label>تایم‌اوت ضبط (ms)</label><input type="number" id="timeout" value="8000"></div>
    <button class="btn btn-settings" onclick="saveConfig()">💾 ذخیره تنظیمات</button>
    <button class="btn btn-info" onclick="sendCmd('restart')">🔄 ری‌استارت دستگاه</button>
  </div>

  <div class="panel" id="panel-ota">
    <div class="ota-note">⚠️ فایل .bin را انتخاب کرده و آپلود کنید. پس از آپلود، دستگاه به‌طور خودکار ری‌استارت می‌شود.</div>
    <form method="POST" action="/update" enctype="multipart/form-data">
      <div class="field"><label>فایل فریمور (.bin)</label><input type="file" name="firmware" accept=".bin"></div>
      <button class="btn btn-ota" type="submit">📤 آپلود و به‌روزرسانی</button>
    </form>
  </div>

  <div id="log"></div>
</div>

<script>
  var ws = new WebSocket('ws://' + location.host + '/ws');

  ws.onopen = function(){ log('WebSocket متصل شد.'); sendCmd('getcfg'); };
  ws.onclose = function(){ log('WebSocket قطع شد.'); setTimeout(()=>location.reload(), 3000); };

  ws.onmessage = function(e){
    log(e.data);
    try{
      var obj = JSON.parse(e.data);
      if(obj.type === 'config'){
        document.getElementById('pinCS').value = obj.pinCS;
        document.getElementById('pinIRQ').value = obj.pinIRQ;
        document.getElementById('pinSDN').value = obj.pinSDN;
        document.getElementById('pinGPIO2').value = obj.pinGPIO2;
        document.getElementById('freq').value = obj.freq;
        document.getElementById('bitrate').value = obj.bitrate;
        document.getElementById('rxBW').value = obj.rxBW;
        document.getElementById('txPower').value = obj.txPower;
        document.getElementById('rssiThr').value = obj.rssiThr;
        document.getElementById('minPulse').value = obj.minPulse;
        document.getElementById('timeout').value = obj.timeout;
      }
      if(obj.pulses !== undefined) document.getElementById('pulseCount').textContent = obj.pulses;
      if(obj.mode === 'auto'){ document.getElementById('btnAuto').classList.add('active'); document.getElementById('btnAuto').textContent = '🔍 در حال شنود...'; }
      else if(obj.mode === 'idle'){ document.getElementById('btnAuto').classList.remove('active'); document.getElementById('btnAuto').textContent = '🔍 حالت شنود خودکار'; }
      if(obj.statusText) document.getElementById('status').innerHTML = 'وضعیت: ' + obj.statusText + ' | پالس: <span class="pulse-count">' + (obj.pulses || 0) + '</span>';
    }catch(ex){}
  };

  function switchTab(name){
    document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
    document.querySelectorAll('.panel').forEach(p=>p.classList.remove('active'));
    document.querySelector('.tab[onclick*="'+name+'"]').classList.add('active');
    document.getElementById('panel-'+name).classList.add('active');
  }

  function sendCmd(cmd){ if(ws.readyState===WebSocket.OPEN) ws.send(cmd); else log('خطا: اتصال برقرار نیست.'); }

  function saveConfig(){
    var cfg = {
      cmd: 'savecfg',
      pinCS: parseInt(document.getElementById('pinCS').value),
      pinIRQ: parseInt(document.getElementById('pinIRQ').value),
      pinSDN: parseInt(document.getElementById('pinSDN').value),
      pinGPIO2: parseInt(document.getElementById('pinGPIO2').value),
      freq: parseFloat(document.getElementById('freq').value),
      bitrate: parseFloat(document.getElementById('bitrate').value),
      rxBW: parseFloat(document.getElementById('rxBW').value),
      txPower: parseInt(document.getElementById('txPower').value),
      rssiThr: parseInt(document.getElementById('rssiThr').value),
      minPulse: parseInt(document.getElementById('minPulse').value),
      timeout: parseInt(document.getElementById('timeout').value)
    };
    ws.send(JSON.stringify(cfg));
  }

  function log(msg){ var el=document.getElementById('log'); el.innerHTML += '> ' + msg + '\n'; el.scrollTop = el.scrollHeight; }
</script>
</body>
</html>
)rawliteral";

// ==================== setup ====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n--- ESP32 + SI4432 SubGHz (حالت AP) ---"));

  loadConfig();

  if (!SPIFFS.begin(true)) {
    Serial.println(F("SPIFFS مقداردهی نشد!"));
    return;
  }

  // ---- حالت AP ----
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.println(F("\n========== WiFi AP =========="));
  Serial.print(F("SSID: ")); Serial.println(AP_SSID);
  Serial.print(F("Pass: ")); Serial.println(AP_PASSWORD);
  Serial.print(F("IP:   ")); Serial.println(WiFi.softAPIP());
  Serial.println(F("============================="));

  // ---- راه‌اندازی رادیو ----
  if (!initRadio()) {
    Serial.println(F("رادیو مقداردهی نشد!"));
  } else {
    enterRxMode();
  }

  // ---- وب سرور ----
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/update", HTTP_POST,
    [](AsyncWebServerRequest *request){
      bool ok = !Update.hasError();
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", ok ? "OK" : "FAIL");
      response->addHeader("Connection", "close");
      request->send(response);
      if (ok) { delay(500); ESP.restart(); }
    },
    handleOTAUpload
  );

  server.begin();
  Serial.println(F("وب سرور راه‌اندازی شد."));
}

// ==================== loop ====================

void loop() {
  ws.cleanupClients();

  if (!capturing && signalDetected && currentMode == MODE_RX && autoMode && radio) {
    signalDetected = false;
    Serial.println("[AUTO] سیگنال شناسایی شد! شروع ضبط...");
    wsLog("سیگنال شناسایی شد! شروع ضبط...");

    uint8_t dummy;
    radio->SPIreadRegister(RADIOLIB_SI443X_REG_INT_STATUS_1, &dummy);
    radio->SPIreadRegister(RADIOLIB_SI443X_REG_INT_STATUS_2, &dummy);

    startCapture();
  }

  if (capturing && (millis() - lastCaptureTime > cfg.timeoutMs)) {
    Serial.println("[AUTO] تایم‌اوت ضبط. توقف...");
    stopCapture();
  }

  delay(5);
}
