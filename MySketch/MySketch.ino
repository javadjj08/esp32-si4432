// ============================================================
// HIZMOS Diagnostic v7.0
// ESP32 + SI4432 + OTA + WiFi
// ============================================================
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RadioLib.h>
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
String si_status = "Not initialized";
int si_error_code = 0;

// ========== صفحه HTML ==========
const char HTML_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="fa" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HIZMOS Diagnostic</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Tahoma,Arial;background:#0a0a0f;color:#e0e0e0;padding:10px;font-size:14px}
.box{max-width:500px;margin:0 auto;background:#12121a;border-radius:14px;padding:14px;border:1px solid #1a1a2e}
h1{color:#00d4ff;font-size:1.1rem;margin-bottom:12px;text-align:center}
h2{color:#00d4ff;font-size:0.95rem;margin:10px 0 6px}
.status{padding:10px;border-radius:8px;margin:6px 0;font-size:0.9rem;line-height:1.5}
.ok{background:#0a1f0a;border:1px solid #0a5;color:#8f8}
.na{background:#1f0a0a;border:1px solid #a00;color:#f88}
.btn{display:block;width:100%;padding:11px;margin:5px 0;border:none;border-radius:8px;background:#1a1a2e;color:#fff;font-size:0.9rem;cursor:pointer;text-align:center}
.btn:hover{background:#2a2a4e}
.btn.blue{background:#06c}
.btn.green{background:#0a5}
.log{background:#000;border:1px solid #1a1a2e;border-radius:6px;padding:9px;font-family:monospace;font-size:0.72rem;color:#8f8;max-height:250px;overflow-y:auto;margin-top:8px;line-height:1.5;direction:ltr;text-align:left;white-space:pre-wrap}
</style>
</head>
<body>
<div class="box">
  <h1>🔬 HIZMOS Diagnostic v7.0</h1>

  <h2>وضعیت SI4432</h2>
  <div class="status" id="siStatus">در حال بررسی...</div>

  <h2>ابزارها</h2>
  <button class="btn blue" onclick="go('/diag_si')">🔍 تست کامل SI4432</button>
  <button class="btn blue" onclick="go('/scan_spectrum')">📊 اسکن طیف ۴۳۰-۴۴۰</button>
  <button class="btn blue" onclick="go('/read_rssi')">📶 خواندن RSSI</button>
  <button class="btn blue" onclick="go('/scan_wifi')">📡 اسکن WiFi</button>
  <button class="btn blue" onclick="go('/sys_info')">ℹ️ اطلاعات سیستم</button>

  <h2>OTA Update</h2>
  <form method="POST" action="/update" enctype="multipart/form-data">
    <input type="file" name="firmware" accept=".bin" style="background:#1a1a2e;color:#fff;padding:8px;border-radius:6px;border:1px solid #2a2a4e;width:100%;margin:5px 0;font-size:0.85rem">
    <button class="btn green" type="submit">📤 آپلود فریمور جدید</button>
  </form>

  <h2>خروجی</h2>
  <div class="log" id="log">آماده...</div>
</div>

<script>
function go(url){
  document.getElementById('log').innerText = 'در حال اجرا...';
  fetch(url).then(r=>r.text()).then(t=>{
    document.getElementById('log').innerText = t;
  }).catch(e=>{
    document.getElementById('log').innerText = 'Error: '+e;
  });
}
function refreshStatus(){
  fetch('/si_status').then(r=>r.json()).then(d=>{
    var el = document.getElementById('siStatus');
    if(d.ok){
      el.className = 'status ok';
      el.innerText = '✅ SI4432 متصل است ('+d.msg+')';
    } else {
      el.className = 'status na';
      el.innerText = '❌ SI4432 شناسایی نشد — '+d.msg;
    }
  }).catch(()=>{});
}
refreshStatus();
setInterval(refreshStatus, 3000);
</script>
</body>
</html>
)HTML";

// ========== تشخیص SI4432 ==========
void init_si4432() {
  Serial.println("\n--- Init SI4432 ---");
  Serial.printf("Pins: CS=%d IRQ=%d SDN=%d SCK=%d MISO=%d MOSI=%d\n",
                SI4432_CS, SI4432_IRQ, SI4432_SDN, HSPI_SCK, HSPI_MISO, HSPI_MOSI);

  pinMode(SI4432_SDN, OUTPUT);
  digitalWrite(SI4432_SDN, LOW);
  delay(100);

  SPI.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, SI4432_CS);
  delay(50);

  Serial.print("Calling radio_si.begin()... ");
  int st = radio_si.begin(si_freq);

  if (st == RADIOLIB_ERR_NONE) {
    Serial.println("SUCCESS");
    si_ok = true;
    si_status = "Initialized OK";
    radio_si.setOutputPower(10);
    radio_si.setBitRate(2.4);
    radio_si.startReceive();
  } else {
    Serial.printf("FAILED code=%d\n", st);
    si_ok = false;
    si_error_code = st;
    si_status = "Error code: " + String(st);
  }
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== HIZMOS Diagnostic v7.0 ===");

  pinMode(SI4432_RX_DATA, INPUT);

  init_si4432();

  WiFi.softAP("HIZMOS-AP", "hizmos123");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](){
    server.send_P(200, "text/html", HTML_PAGE);
  });

  server.on("/si_status", HTTP_GET, [](){
    String j = "{\"ok\":" + String(si_ok ? "true" : "false");
    j += ",\"msg\":\"" + si_status + "\"";
    j += ",\"code\":" + String(si_error_code) + "}";
    server.send(200, "application/json", j);
  });

  server.on("/diag_si", HTTP_GET, [](){
    String r = "=== SI4432 Diagnostic ===\n";
    r += "Pins used:\n";
    r += "  CS    -> GPIO " + String(SI4432_CS) + "\n";
    r += "  IRQ   -> GPIO " + String(SI4432_IRQ) + "\n";
    r += "  SDN   -> GPIO " + String(SI4432_SDN) + "\n";
    r += "  SCK   -> GPIO " + String(HSPI_SCK) + "\n";
    r += "  MISO  -> GPIO " + String(HSPI_MISO) + "\n";
    r += "  MOSI  -> GPIO " + String(HSPI_MOSI) + "\n";
    r += "  RX    -> GPIO " + String(SI4432_RX_DATA) + "\n\n";
    r += "SDN voltage: " + String(digitalRead(SI4432_SDN)) + " (should be 0)\n";
    r += "IRQ voltage: " + String(digitalRead(SI4432_IRQ)) + " (should be 1 if pull-up)\n\n";
    r += "radio_si.begin() result: ";
    if (si_ok) {
      r += "OK\n";
      r += "Version: " + String(radio_si.getChipVersion()) + "\n";
      r += "Frequency: " + String(si_freq, 2) + " MHz\n";
      r += "RSSI: " + String(radio_si.getRSSI(), 1) + " dBm\n";
    } else {
      r += "FAIL (code " + String(si_error_code) + ")\n";
    }
    server.send(200, "text/plain", r);
  });

  server.on("/scan_spectrum", HTTP_GET, [](){
    if (!si_ok) { server.send(200, "text/plain", "SI4432 not available"); return; }
    String r = "=== Spectrum 430-440 MHz ===\n";
    for (int f = 430; f <= 440; f++) {
      radio_si.setFrequency(f);
      radio_si.startReceive();
      delay(40);
      r += String(f) + " MHz : " + String(radio_si.getRSSI(), 1) + " dBm\n";
    }
    radio_si.setFrequency(si_freq);
    radio_si.startReceive();
    server.send(200, "text/plain", r);
  });

  server.on("/read_rssi", HTTP_GET, [](){
    if (!si_ok) { server.send(200, "text/plain", "SI4432 not available"); return; }
    radio_si.startReceive();
    delay(20);
    String r = "RSSI @ " + String(si_freq, 2) + " MHz : ";
    r += String(radio_si.getRSSI(), 1) + " dBm\n";
    server.send(200, "text/plain", r);
  });

  server.on("/scan_wifi", HTTP_GET, [](){
    int n = WiFi.scanNetworks();
    String r = "=== WiFi Scan (" + String(n) + ") ===\n";
    for (int i = 0; i < n; i++) {
      r += String(i+1) + ". " + WiFi.SSID(i) + " (" + WiFi.RSSI(i) + " dBm) CH" + WiFi.channel(i) + "\n";
    }
    server.send(200, "text/plain", r);
  });

  server.on("/sys_info", HTTP_GET, [](){
    String r = "=== System Info ===\n";
    r += "Chip: " + String(ESP.getChipModel()) + "\n";
    r += "CPU: " + String(ESP.getCpuFreqMHz()) + " MHz\n";
    r += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes\n";
    r += "Flash Size: " + String(ESP.getFlashChipSize()) + " bytes\n";
    r += "Sketch Size: " + String(ESP.getSketchSize()) + " bytes\n";
    r += "Free Sketch: " + String(ESP.getFreeSketchSpace()) + " bytes\n";
    r += "Uptime: " + String(millis()/1000) + " s\n";
    server.send(200, "text/plain", r);
  });

  server.on("/update", HTTP_POST, [](){
    server.send(200, "text/plain", Update.hasError() ? "FAIL" : "OK - Rebooting...");
    delay(1000);
    ESP.restart();
  }, [](){
    HTTPUpload& up = server.upload();
    if (up.status == UPLOAD_FILE_START) {
      Serial.printf("OTA: %s\n", up.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (up.status == UPLOAD_FILE_WRITE) {
      if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
    } else if (up.status == UPLOAD_FILE_END) {
      if (Update.end(true)) Serial.printf("OTA OK: %u bytes\n", up.totalSize);
      else Update.printError(Serial);
    }
  });

  server.begin();
  Serial.println("=== Ready ===");
}

void loop() {
  server.handleClient();
  delay(1);
}
