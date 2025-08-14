/*
 * ESP32-S3 FTMS Controller (Bluedroid / standard ESP32 BLE)
 * - BLE Central (Client) to FTMS treadmill using BLEDevice.h (Bluedroid)
 * - Web UI with direct FIT upload (browser parses FIT -> JSON steps)
 * - Stores workouts in LittleFS
 * - Sends FTMS Set Target Speed / Set Target Incline, Start/Pause/Stop
 *
 * Notes:
 *  - Uses standard ESP32 BLE library (NOT NimBLE)
 *  - Supports time-based steps with speed (km/h) and incline (%)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <vector>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

// -------- Standard ESP32 BLE (Bluedroid) ----------
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>

// ========================= WiFi CONFIG =========================
const char* WIFI_SSID     = "chicago";
const char* WIFI_PASSWORD = "Neukirch20211!";

// ========================= FTMS UUIDs ==========================
static BLEUUID UUID_FTMS_SERVICE((uint16_t)0x1826);
static BLEUUID UUID_TREADMILL_DATA((uint16_t)0x2ACD);
static BLEUUID UUID_FTMS_STATUS((uint16_t)0x2ADA);
static BLEUUID UUID_CONTROL_POINT((uint16_t)0x2AD9);

// FTMS opcodes
#define FTMS_REQUEST_CONTROL        0x00
#define FTMS_RESET                  0x01
#define FTMS_SET_TARGET_SPEED       0x02    // value: uint16 0.01 km/h
#define FTMS_SET_TARGET_INCLINATION 0x03    // value: sint16 0.1 %
#define FTMS_START_RESUME           0x07
#define FTMS_STOP_PAUSE             0x08    // param: 0x01 stop, 0x02 pause (common impl)

// ========================= SERVER ==============================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ========================= WORKOUT MODEL =======================
struct Step {
  uint32_t sec;       // duration seconds
  float speedKmh;     // 0.01 km/h resolution in FTMS
  float inclinePct;   // 0.1% resolution in FTMS
};

static std::vector<Step> workout;
static String currentWorkoutName = "";
static bool running = false;
static bool paused  = false;
static uint32_t stepIdx = 0;
static uint32_t stepElapsed = 0; // seconds within current step
static uint32_t workoutElapsed = 0;
static uint32_t lastTickMs = 0;

// ========================= BLE GLOBALS (Bluedroid) =============
static bool ftmsReady = false;
static BLEAddress foundAddr = BLEAddress((uint8_t*)"\0\0\0\0\0\0");
static bool haveFtmsAddr = false;

static BLEClient* pClient = nullptr;
static BLERemoteService* pFTMS = nullptr;
static BLERemoteCharacteristic* pCtrl = nullptr;
static BLERemoteCharacteristic* pStatus = nullptr;
static BLERemoteCharacteristic* pTreadmillData = nullptr;

// ========================= UI HTML (with FIT parser) ==========
const char* INDEX_HTML = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>FTMS Treadmill Controller</title>
<style>
body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Arial;margin:20px}
.card{border:1px solid #ddd;border-radius:12px;padding:16px;max-width:900px;margin:auto;box-shadow:0 1px 8px rgba(0,0,0,.06)}
h1{margin:0 0 12px}
section{margin:16px 0}
table{border-collapse:collapse;width:100%} td,th{border-bottom:1px solid #eee;padding:8px;text-align:left}
.btn{display:inline-block;padding:10px 16px;border-radius:10px;border:0;background:#007aff;color:#fff;margin-right:8px;cursor:pointer}
.btn.red{background:#e74c3c} .btn.gray{background:#999}
.badge{display:inline-block;background:#eee;color:#333;padding:3px 8px;border-radius:8px;margin-left:6px}
.progress{height:10px;background:#eee;border-radius:8px;overflow:hidden}
.progress>div{height:10px;background:#007aff;width:0%}
.small{color:#666;font-size:.9em}
</style></head><body>
<div class="card">
<h1>FTMS Treadmill Controller</h1>

<section>
  <h3>Upload TrainingPeaks FIT</h3>
  <input id="fitfile" type="file" accept=".fit">
  <button class="btn" onclick="uploadFIT()">Upload & Load</button>
  <div class="small">Pick a TP structured workout (*.fit). It's parsed locally and sent to the device.</div>
</section>

<section>
  <h3>Workouts on device</h3>
  <table id="files"><thead><tr><th>Name</th><th>Size</th><th></th></tr></thead><tbody></tbody></table>
</section>

<section>
  <h3>Status <span id="status" class="badge">Idle</span></h3>
  <div>Workout: <b id="wname">—</b></div>
  <div>Step <span id="sidx">0</span> / <span id="stotal">0</span></div>
  <div>Current: <span id="curr">—</span></div>
  <div>Next: <span id="next">—</span></div>
  <div>Elapsed: <span id="welapsed">00:00:00</span></div>
  <div class="progress"><div id="pbar"></div></div>
  <div style="margin-top:10px">
    <button class="btn" onclick="api('/start')">Start</button>
    <button class="btn gray" onclick="api('/pause')">Pause</button>
    <button class="btn red" onclick="api('/stop')">Stop</button>
    <button class="btn gray" onclick="api('/scan')">Scan & Connect FTMS</button>
  </div>
</section>
</div>

<script>
// ----------------- Helpers -----------------
async function listFiles(){
  const t=document.querySelector('#files tbody'); t.innerHTML='';
  const r=await fetch('/list'); const js=await r.json();
  js.files.forEach(f=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${f.name}</td><td>${f.size}</td>
      <td><button class="btn" onclick="load('${f.name}')">Load</button>
      <button class="btn red" onclick="delFile('${f.name}')">Delete</button></td>`;
    t.appendChild(tr);
  });
}
async function load(n){
  const r=await fetch('/load?name='+encodeURIComponent(n));
  alert(await r.text()); await listFiles();
}
async function delFile(n){
  if(!confirm('Delete '+n+'?'))return;
  const r=await fetch('/delete?name='+encodeURIComponent(n)); alert(await r.text()); listFiles();
}
async function api(p){ const r=await fetch(p); alert(await r.text()); }

const ws=new WebSocket(`ws://${location.host}/ws`);
ws.onmessage=(ev)=>{
  const d=JSON.parse(ev.data);
  if(d.workout!==undefined) document.getElementById('wname').textContent=d.workout||'—';
  if(d.running!==undefined) document.getElementById('status').textContent=d.running?(d.paused?'Paused':'Running'):'Idle';
  if(d.stepIdx!==undefined){
    document.getElementById('sidx').textContent=d.stepIdx+1;
    document.getElementById('stotal').textContent=(d.totalSteps||0);
  }
  if(d.curr){
    document.getElementById('curr').textContent=`${d.curr.sec}s @ ${d.curr.speed.toFixed(1)} km/h, ${d.curr.incline.toFixed(1)}%`;
  }
  if(d.next){
    document.getElementById('next').textContent=`${d.next.sec}s @ ${d.next.speed.toFixed(1)} km/h, ${d.next.incline.toFixed(1)}%`;
  } else document.getElementById('next').textContent='—';
  if(d.workoutElapsed!==undefined){
    document.getElementById('welapsed').textContent=new Date(d.workoutElapsed*1000).toISOString().substring(11,19);
  }
  if(d.curr && d.stepElapsed!==undefined){
    let pct = Math.min(100, Math.max(0, d.stepElapsed*100/d.curr.sec));
    document.getElementById('pbar').style.width=pct+'%';
  } else document.getElementById('pbar').style.width='0%';
};
listFiles();

// ----------------- Minimal FIT parser (Workout Steps only) -----------------
// Supports common TP exports: global 27 (workout_step)
// duration_type = 0 (time seconds), duration_value = sec
// target_type = 0x0D (speed, m/s*1000) or 0x0F (grade, %*100)
function uploadFIT(){
  const inp = document.getElementById('fitfile');
  if(!inp.files || !inp.files[0]) { alert('Choose a FIT file first'); return; }
  const file = inp.files[0];
  const fr = new FileReader();
  fr.onload = async () => {
    try {
      const steps = parseFitToSteps(new Uint8Array(fr.result));
      if(!steps.length){ alert('No runnable steps parsed'); return; }
      const payload = { name: file.name.replace(/\.fit$/i,''), steps: steps };
      const res = await fetch('/upload_workout', {
        method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify(payload)
      });
      alert(await res.text());
      listFiles();
    } catch(e){
      console.error(e);
      alert('FIT parse failed: '+e.message);
    }
  };
  fr.readAsArrayBuffer(file);
}

function parseFitToSteps(bytes){
  const headerSize = bytes[0];
  if(headerSize < 12) throw new Error('Invalid FIT header');
  const dataSize = bytes[4] | (bytes[5]<<8) | (bytes[6]<<16) | (bytes[7]<<24);
  const dataStart = headerSize;
  const dataEnd = dataStart + dataSize;
  const defs = {};
  const steps = [];
  let i = dataStart;

  while(i < dataEnd){
    const hdr = bytes[i++];
    const isDef = (hdr & 0x40) !== 0;
    const localNum = hdr & 0x0F;

    if(isDef){
      const arch = bytes[i++]; // assume little endian
      const gmn = bytes[i] | (bytes[i+1]<<8); i+=2;
      const numFields = bytes[i++];
      const fields = [];
      for(let f=0; f<numFields; f++){
        const num = bytes[i++]; const size = bytes[i++]; const base = bytes[i++];
        fields.push({num, size, base});
      }
      defs[localNum] = {globalNum: gmn, fields};
    } else {
      const def = defs[localNum];
      if(!def) throw new Error('Data with no definition');
      const start = i;
      let values = {};
      for(const f of def.fields){
        const v = readField(bytes, i, f.size);
        values[f.num] = v;
        i += f.size;
      }
      if(def.globalNum === 27){ // workout_step
        const durationType = values[3];
        const durationValue = values[4];
        const targetType = values[5];
        const low = values[7];
        const high = values[8];

        let sec = 0, speed = NaN, incline = NaN;
        if(durationType === 0) sec = durationValue >>> 0;

        if(targetType === 13){ // speed m/s*1000
          const msLow = low/1000.0, msHigh = high/1000.0;
          const ms = (isFinite(msLow) && isFinite(msHigh) && (msLow>0 || msHigh>0)) ? (msLow+msHigh)/2 : NaN;
          if(isFinite(ms)) speed = ms * 3.6;
        } else if (targetType === 15){ // grade %*100
          const gLow = low/100.0, gHigh = high/100.0;
          const g = (isFinite(gLow) && isFinite(gHigh)) ? (gLow+gHigh)/2 : NaN;
          if(isFinite(g)) incline = g;
        }

        if(!isFinite(speed)) speed = 10.0;
        if(!isFinite(incline)) incline = 0.0;
        if(sec>0) steps.push({sec:sec, speed:round1(speed), incline:round1(incline)});
      }
      if(i <= start) throw new Error('Parser stuck');
    }
  }
  return steps;

  function round1(x){ return Math.round(x*10)/10; }
  function readField(buf, off, size){
    let val = 0;
    switch(size){
      case 1: val = buf[off]; break;
      case 2: val = buf[off] | (buf[off+1]<<8); break;
      case 4: val = (buf[off] | (buf[off+1]<<8) | (buf[off+2]<<16) | (buf[off+3]<<24)) >>> 0; break;
      default: val = 0; for(let k=0;k<size;k++) val |= (buf[off+k]<<(8*k));
    }
    return val;
  }
}
</script>
</body></html>
)HTML";

// ========================= SMALL UTILS =========================
void wsBroadcastState(const char* evt = nullptr) {
  StaticJsonDocument<512> doc;
  if (evt) doc["event"] = evt;
  doc["workout"] = currentWorkoutName;
  doc["running"] = running;
  doc["paused"]  = paused;
  doc["stepIdx"] = stepIdx;
  doc["stepElapsed"] = stepElapsed;
  doc["workoutElapsed"] = workoutElapsed;
  if (!workout.empty() && stepIdx < workout.size()) {
    doc["curr"]["sec"] = workout[stepIdx].sec;
    doc["curr"]["speed"] = workout[stepIdx].speedKmh;
    doc["curr"]["incline"] = workout[stepIdx].inclinePct;
  }
  if (stepIdx + 1 < workout.size()) {
    doc["next"]["sec"] = workout[stepIdx+1].sec;
    doc["next"]["speed"] = workout[stepIdx+1].speedKmh;
    doc["next"]["incline"] = workout[stepIdx+1].inclinePct;
  }
  String out; serializeJson(doc, out);
  ws.textAll(out);
}

// ========================= FTMS WRITES =========================
static bool ftmsWriteCtrl(const uint8_t* data, size_t len, bool withResponse = true) {
  if (!pCtrl) return false;
  // Use the pointer overload available in all versions:
  return pCtrl->writeValue(const_cast<uint8_t*>(data), len, withResponse);
}
static bool ftmsRequestControl() {
  uint8_t pkt[1] = { FTMS_REQUEST_CONTROL };
  return ftmsWriteCtrl(pkt, sizeof(pkt));
}
static bool ftmsStart() {
  uint8_t pkt[2] = { FTMS_START_RESUME, 0x01 }; // commonly 0x01=start
  return ftmsWriteCtrl(pkt, sizeof(pkt));
}
static bool ftmsStop() {
  uint8_t pkt[2] = { FTMS_STOP_PAUSE, 0x01 }; // stop
  return ftmsWriteCtrl(pkt, sizeof(pkt));
}
static bool ftmsPause() {
  uint8_t pkt[2] = { FTMS_STOP_PAUSE, 0x02 }; // pause
  return ftmsWriteCtrl(pkt, sizeof(pkt));
}
static bool ftmsSetSpeed(float kmh) {
  int32_t v = (int32_t) lroundf(kmh * 100.0f);            // 0.01 km/h
  if (v < 0) v = 0; if (v > 0xFFFF) v = 0xFFFF;
  uint8_t pkt[3] = { FTMS_SET_TARGET_SPEED, (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
  return ftmsWriteCtrl(pkt, sizeof(pkt));
}
static bool ftmsSetIncline(float pct) {
  int32_t v = (int32_t) lroundf(pct * 10.0f);             // 0.1 %
  if (v < -32768) v = -32768; if (v > 32767) v = 32767;
  uint8_t pkt[3] = { FTMS_SET_TARGET_INCLINATION, (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
  return ftmsWriteCtrl(pkt, sizeof(pkt));
}

// Apply a step immediately
static void applyCurrentStep() {
  if (workout.empty() || stepIdx >= workout.size() || !ftmsReady) return;
  Step &s = workout[stepIdx];
  ftmsSetSpeed(s.speedKmh);
  delay(20);
  ftmsSetIncline(s.inclinePct);
  wsBroadcastState("step_applied");
}

// ========================= BLE Notify (optional) ===============
static void statusNotifyCB(
  BLERemoteCharacteristic* /*c*/, uint8_t* data, size_t len, bool /*isNotify*/) {
  Serial.print("FTMS Status: ");
  for (size_t i=0;i<len;i++) Serial.printf("%02X ", data[i]);
  Serial.println();
}

// ========================= BLE SCAN CALLBACKS ==================
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(UUID_FTMS_SERVICE)) {
      Serial.printf("Found FTMS: %s\n", advertisedDevice.toString().c_str());
      foundAddr = advertisedDevice.getAddress();
      haveFtmsAddr = true;
      BLEDevice::getScan()->stop();
    }
  }
};

// ========================= BLE CONNECT / DISCOVER =============
static bool connectFTMS() {
  if (!haveFtmsAddr) return false;
  if (pClient && pClient->isConnected()) return true;

  pClient = BLEDevice::createClient();
  if (!pClient->connect(foundAddr)) {
    Serial.println("BLE connect failed");
    pClient->disconnect();
    pClient = nullptr;
    return false;
  }

  pFTMS = pClient->getService(UUID_FTMS_SERVICE);
  if (!pFTMS) { Serial.println("No FTMS service"); return false; }

  pCtrl = pFTMS->getCharacteristic(UUID_CONTROL_POINT);
  pStatus = pFTMS->getCharacteristic(UUID_FTMS_STATUS);
  pTreadmillData = pFTMS->getCharacteristic(UUID_TREADMILL_DATA);

  if (!pCtrl) { Serial.println("No Control Point"); return false; }

  if (pStatus && pStatus->canNotify()) {
    pStatus->registerForNotify(statusNotifyCB);
  }
  if (pTreadmillData && pTreadmillData->canNotify()) {
    pTreadmillData->registerForNotify(nullptr);
  }

  // Request control and start
  ftmsRequestControl();
  delay(80);
  ftmsStart();
  ftmsReady = true;
  Serial.println("FTMS ready");
  return true;
}

static void doScanForFTMS(uint32_t seconds = 8) {
  haveFtmsAddr = false;
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), false);
  scan->setInterval(45);
  scan->setWindow(30);
  scan->setActiveScan(true);
  Serial.println("Scanning for FTMS...");
  scan->start(seconds, false);
  if (!haveFtmsAddr) Serial.println("No FTMS device found.");
  scan->stop();
}

// ========================= WORKOUT LOADER ======================
static bool parseJSON(fs::FS& fs, const char* path) {
  File f = fs.open(path, "r");
  if (!f) return false;
  StaticJsonDocument<8192> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;
  workout.clear();
  JsonArray stepsArr = doc["steps"].as<JsonArray>();
  for (JsonObject s : stepsArr) {
    uint32_t sec = s["sec"] | 0;
    float spd = s["speed"] | 0.0;
    float inc = s["incline"] | 0.0;
    if (sec>0) workout.push_back({sec, spd, inc});
  }
  const char* nm = doc["name"] | "Workout";
  currentWorkoutName = String(nm);
  return !workout.empty();
}

static bool loadWorkout(const String& filename) {
  String lower = filename; lower.toLowerCase();
  if (!lower.endsWith(".json")) return false; // store parsed FIT as JSON
  bool ok = parseJSON(LittleFS, filename.c_str());
  if (ok) { stepIdx = 0; stepElapsed = 0; workoutElapsed = 0; }
  return ok;
}

// ========================= HTTP HANDLERS =======================
static void handleList(AsyncWebServerRequest* req) {
  StaticJsonDocument<4096> doc;
  JsonArray arr = doc.createNestedArray("files");
  File root = LittleFS.open("/", "r");
  File f = root.openNextFile();
  while (f) {
    JsonObject o = arr.createNestedObject();
    o["name"] = String(f.name());
    o["size"] = (uint32_t)f.size();
    f = root.openNextFile();
  }
  String out; serializeJson(doc, out);
  req->send(200, "application/json", out);
}

static void handleLoad(AsyncWebServerRequest* req) {
  if (!req->hasParam("name")) { req->send(400, "text/plain", "name required"); return; }
  String name = req->getParam("name")->value();
  if (!loadWorkout(name)) { req->send(400, "text/plain", "Load failed / unsupported file"); return; }
  StaticJsonDocument<64> d; d["totalSteps"] = (uint32_t)workout.size();
  String out; serializeJson(d, out); ws.textAll(out);
  req->send(200, "text/plain", "Loaded " + name);
}

static void handleDelete(AsyncWebServerRequest* req) {
  if (!req->hasParam("name")) { req->send(400, "text/plain", "name required"); return; }
  String name = req->getParam("name")->value();
  if (!name.startsWith("/")) name = "/" + name;
  bool ok = LittleFS.remove(name);
  req->send(ok?200:404, "text/plain", ok?"Deleted":"Not found");
}

static void handleStart(AsyncWebServerRequest* req) {
  if (workout.empty()) { req->send(400, "text/plain", "No workout loaded"); return; }
  if (!ftmsReady) { req->send(400, "text/plain", "FTMS not connected"); return; }
  running = true; paused = false;
  lastTickMs = millis();
  wsBroadcastState("start");
  applyCurrentStep();
  req->send(200, "text/plain", "Started");
}

static void handlePause(AsyncWebServerRequest* req) {
  if (!running) { req->send(400, "text/plain", "Not running"); return; }
  paused = !paused;
  if (paused) { ftmsPause(); }
  else { ftmsStart(); applyCurrentStep(); }
  wsBroadcastState("pause_toggle");
  req->send(200, "text/plain", paused?"Paused":"Resumed");
}

static void handleStop(AsyncWebServerRequest* req) {
  running = false; paused = false;
  stepIdx = 0; stepElapsed = 0; workoutElapsed = 0;
  ftmsStop();
  wsBroadcastState("stop");
  req->send(200, "text/plain", "Stopped");
}

// Accepts: { name: "WorkoutName", steps: [ {sec:.., speed:.., incline:..}, ... ] }
static void handleUploadWorkout(AsyncWebServerRequest* req, uint8_t* data, size_t len) {
  StaticJsonDocument<16384> doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) { req->send(400, "text/plain", "Bad JSON"); return; }

  const char* nm2 = doc["name"] | "workout";
  String name = nm2;
  JsonArray arr = doc["steps"].as<JsonArray>();
  if (arr.isNull() || arr.size()==0) {
    req->send(400, "text/plain", "No steps");
    return;
  }

  String file = "/" + name + String(".json");
  File f = LittleFS.open(file, "w");
  if (!f) { req->send(500, "text/plain", "FS open fail"); return; }
  serializeJson(doc, f);
  f.close();

  bool ok = loadWorkout(file);
  if (!ok) { req->send(500, "text/plain", "Loaded file invalid"); return; }

  StaticJsonDocument<64> d; d["totalSteps"] = (uint32_t)workout.size();
  String out; serializeJson(d, out); ws.textAll(out);

  req->send(200, "text/plain", "FIT uploaded, parsed and loaded: " + file);
}

// ========================= WIFI/WEB INIT =======================
static void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setHostname("esp32s3-ftms");
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi:");
  for(int i=0;i<30 && WiFi.status()!=WL_CONNECTED;i++){ delay(200); Serial.print("."); }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
}

static void initWeb() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){ r->send(200, "text/html", INDEX_HTML); });
  server.on("/list", HTTP_GET, handleList);
  server.on("/load", HTTP_GET, handleLoad);
  server.on("/delete", HTTP_GET, handleDelete);
  server.on("/start", HTTP_GET, handleStart);
  server.on("/pause", HTTP_GET, handlePause);
  server.on("/stop",  HTTP_GET, handleStop);
  server.on("/scan",  HTTP_GET, [](AsyncWebServerRequest* r){
    doScanForFTMS(8);
    r->send(200, "text/plain", haveFtmsAddr ? ("Found FTMS: " + foundAddr.toString()) : "No FTMS found");
  });

  // JSON workout upload (from browser FIT parser)
  server.on("/upload_workout", HTTP_POST,
    [](AsyncWebServerRequest* r){ /* response is sent in body handler */ },
    NULL,
    [](AsyncWebServerRequest* r, uint8_t* data, size_t len, size_t index, size_t total) {
      static std::vector<uint8_t> body;
      if (index == 0) body.clear();
      body.insert(body.end(), data, data+len);
      if (index + len == total) {
        handleUploadWorkout(r, body.data(), body.size());
        body.clear();
      }
    }
  );

  ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType, void*, uint8_t*, size_t){});
  server.addHandler(&ws);
  server.begin();
  Serial.println("Web server started");
}

// ========================= LOOP ENGINE =========================
static void tickEngine() {
  if (!running || paused || workout.empty() || !ftmsReady) return;
  uint32_t now = millis();
  if (now - lastTickMs < 1000) return;
  lastTickMs = now;

  stepElapsed++;
  workoutElapsed++;

  // step change?
  if (stepElapsed >= workout[stepIdx].sec) {
    stepIdx++;
    stepElapsed = 0;
    if (stepIdx >= workout.size()) {
      running = false;
      ftmsStop();
      wsBroadcastState("finished");
      return;
    }
    applyCurrentStep();
  }

  // push totals to UI occasionally
  StaticJsonDocument<64> d; d["totalSteps"] = (uint32_t)workout.size();
  String out; serializeJson(d, out); ws.textAll(out);
  wsBroadcastState("tick");
}

// ========================= SETUP / LOOP ========================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nFTMS Controller boot (Bluedroid)");

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }

  initWiFi();

  // Init classic ESP32 BLE (Bluedroid)
  BLEDevice::init("ESP32S3-FTMS-Client");
  BLEDevice::setPower(ESP_PWR_LVL_P9); // adjust if needed

  initWeb();

  // optional: auto-scan on boot
  doScanForFTMS(8);
}

void loop() {
  tickEngine();

  // try connect when a device address has been found
  static uint32_t lastConnTry = 0;
  if (haveFtmsAddr && !ftmsReady && millis() - lastConnTry > 2000) {
    lastConnTry = millis();
    if (!connectFTMS()) {
      // will retry later
    }
  }

  // if connection drops, reset state
  if (pClient && !pClient->isConnected() && ftmsReady) {
    Serial.println("BLE disconnected");
    ftmsReady = false;
    pClient = nullptr;
  }
}