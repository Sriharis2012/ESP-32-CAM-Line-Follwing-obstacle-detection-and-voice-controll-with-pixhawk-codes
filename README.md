#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// Optional ToF
typedef bool tof_enabled_t; // for readability
#if __has_include(<Adafruit_VL53L0X.h>)
  #include <Wire.h>
  #include <Adafruit_VL53L0X.h>
  Adafruit_VL53L0X lox = Adafruit_VL53L0X();
  tof_enabled_t TOF_AVAILABLE = true;
#else
  tof_enabled_t TOF_AVAILABLE = false;
#endif

// MAVLink (install library so this resolves)
extern "C" {
  #include <mavlink.h>
}

/********** Config **********/
// Wi‑Fi
const char* WIFI_SSID = "ESP32CAM_Drone";
const char* WIFI_PASS = "esp32pixhawk"; // change me

// TELEM UART
#define UART_PORT Serial2
constexpr int PIN_TX2 = 17; // GPIO17 -> Pixhawk RX
constexpr int PIN_RX2 = 16; // GPIO16 -> Pixhawk TX
constexpr uint32_t BAUD_TELEM = 57600;

// I2C (for ToF)
constexpr int I2C_SDA = 15;
constexpr int I2C_SCL = 14;

// Camera pins for AI Thinker
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Behavior
bool g_line_follow_enabled = false;
bool g_obstacle_avoid_enabled = true;
bool g_armed = false;
uint8_t g_base_throttle = 1500; // RC mid = 1500. Tune to your airframe & flight mode.

// Safety
const uint16_t RC_MIN = 1100, RC_MID = 1500, RC_MAX = 1900;
const float OBSTACLE_M = 1.8f; // meters — stop if closer than this

/********** Globals **********/
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// A tiny struct of state we broadcast to UI
struct UiState { float line_err; float tof_m; bool armed; bool lf; bool oa; };
UiState g_state{0, NAN, false, false, true};

/********** Utils **********/
static inline uint32_t micros32() { return (uint32_t)micros(); }

/********** MAVLink helpers **********/
// Replace with your system/component ids if needed
constexpr uint8_t SYS_ID = 42;      // ESP32 companion sys id
constexpr uint8_t COMP_ID = MAV_COMP_ID_ONBOARD_COMPUTER;
constexpr uint8_t TARGET_SYS = 1;   // Pixhawk autopilot typically 1
constexpr uint8_t TARGET_COMP = 1;  // Autopilot comp id

void mav_send(const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  UART_PORT.write(buf, len);
}

void mav_send_heartbeat() {
  mavlink_message_t msg; 
  mavlink_heartbeat_t hb{}; 
  hb.type = MAV_TYPE_ONBOARD_CONTROLLER; 
  hb.autopilot = MAV_AUTOPILOT_INVALID; 
  hb.base_mode = 0; 
  hb.custom_mode = 0; 
  hb.system_status = MAV_STATE_ACTIVE; 
  hb.mavlink_version = 3; 
  mavlink_msg_heartbeat_encode(SYS_ID, COMP_ID, &msg, &hb);
  mav_send(msg);
}

void mav_cmd_long(uint16_t cmd, float p1=0,float p2=0,float p3=0,float p4=0,float p5=0,float p6=0,float p7=0) {
  mavlink_message_t msg; 
  mavlink_command_long_t cl{}; 
  cl.target_system = TARGET_SYS; 
  cl.target_component = TARGET_COMP; 
  cl.command = cmd; 
  cl.confirmation = 0; 
  cl.param1=p1; cl.param2=p2; cl.param3=p3; cl.param4=p4; cl.param5=p5; cl.param6=p6; cl.param7=p7; 
  mavlink_msg_command_long_encode(SYS_ID, COMP_ID, &msg, &cl);
  mav_send(msg);
}

void mav_set_mode(uint32_t custom_mode, uint8_t base_mode=0) {
  // ArduPilot: use MAV_CMD_DO_SET_MODE with base+custom
  mav_cmd_long(MAV_CMD_DO_SET_MODE, base_mode, custom_mode, 0, 0, 0, 0, 0);
}

void mav_arm(bool arm) {
  // ArduPilot: MAV_CMD_COMPONENT_ARM_DISARM param1: 1 arm, 0 disarm
  mav_cmd_long(MAV_CMD_COMPONENT_ARM_DISARM, arm ? 1 : 0, 0, 0, 0, 0, 0, 0);
}

void mav_rc_override(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4) {
  mavlink_message_t msg; 
  mavlink_rc_channels_override_t rc{}; 
  rc.target_system = TARGET_SYS; 
  rc.target_component = TARGET_COMP; 
  rc.chan1_raw = ch1; // roll
  rc.chan2_raw = ch2; // pitch
  rc.chan3_raw = ch3; // throttle
  rc.chan4_raw = ch4; // yaw
  // leave others 0 (ignored)
  mavlink_msg_rc_channels_override_encode(SYS_ID, COMP_ID, &msg, &rc);
  mav_send(msg);
}

/********** Camera init **********/
bool camera_init_ok = false;

bool init_camera() {
  camera_config_t config; 
  config.ledc_channel = LEDC_CHANNEL_0; 
  config.ledc_timer = LEDC_TIMER_0; 
  config.pin_d0 = Y2_GPIO_NUM; 
  config.pin_d1 = Y3_GPIO_NUM; 
  config.pin_d2 = Y4_GPIO_NUM; 
  config.pin_d3 = Y5_GPIO_NUM; 
  config.pin_d4 = Y6_GPIO_NUM; 
  config.pin_d5 = Y7_GPIO_NUM; 
  config.pin_d6 = Y8_GPIO_NUM; 
  config.pin_d7 = Y9_GPIO_NUM; 
  config.pin_xclk = XCLK_GPIO_NUM; 
  config.pin_pclk = PCLK_GPIO_NUM; 
  config.pin_vsync = VSYNC_GPIO_NUM; 
  config.pin_href = HREF_GPIO_NUM; 
  config.pin_sscb_sda = SIOD_GPIO_NUM; 
  config.pin_sscb_scl = SIOC_GPIO_NUM; 
  config.pin_pwdn = PWDN_GPIO_NUM; 
  config.pin_reset = RESET_GPIO_NUM; 
  config.xclk_freq_hz = 20000000; 
  config.pixel_format = PIXFORMAT_JPEG; 

  if(psramFound()) {
    config.frame_size = FRAMESIZE_QVGA; // 320x240 (good balance for processing)
    config.jpeg_quality = 12;          // lower = better quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QQVGA; // 160x120
    config.jpeg_quality = 20;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) return false;

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA); // ensure 320x240 for processing
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);
  s->set_special_effect(s, 0);
  s->set_whitebal(s, 1);
  return true;
}

/********** Line detection (simple) **********/
// We re-grab a grayscale copy via camera->get(); then do crude threshold and centroid on bottom band.
float compute_line_error_from_rgb888(uint8_t* rgb, int w, int h) {
  // Consider bottom 1/3rd band
  int y0 = (h*2)/3;
  int y1 = h;
  uint32_t sumx = 0; uint32_t count = 0;
  for (int y = y0; y < y1; ++y) {
    for (int x = 0; x < w; ++x) {
      uint8_t r = rgb[(y*w + x)*3 + 0];
      uint8_t g = rgb[(y*w + x)*3 + 1];
      uint8_t b = rgb[(y*w + x)*3 + 2];
      // Luminance
      uint8_t Y = (uint8_t)((77*r + 150*g + 29*b) >> 8);
      bool is_dark = (Y < 60); // threshold for dark line; tune.
      if (is_dark) { sumx += x; count++; }
    }
  }
  if (count < 10) return NAN; // no line found
  float cx = (float)sumx / (float)count; // centroid x
  float err = (cx - (w*0.5f)) / (w*0.5f); // -1..+1 left/right
  return err;
}

/********** MJPEG streamer **********/
// For simplicity we use built-in JPEG frames; for line detection we occasionally decode to RGB via built-in helper.

void handle_stream(AsyncWebServerRequest *request) {
  AsyncWebServerResponse *response = request->beginChunkedResponse("multipart/x-mixed-replace; boundary=frame", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) return 0;

    // Build multipart frame
    static const char* BOUNDARY = "--frame
Content-Type: image/jpeg
Content-Length: %u

";
    int headerLen = snprintf((char*)buffer, maxLen, BOUNDARY, fb->len);
    if ((size_t)headerLen > maxLen) { esp_camera_fb_return(fb); return 0; }

    // If there is room, copy header + image; else signal to send header now, image later
    if (maxLen < (size_t)headerLen + fb->len + 2) {
      memcpy(buffer, buffer, headerLen);
      esp_camera_fb_return(fb);
      return headerLen; // chunk header now; next call will send image
    }
    memcpy(buffer + headerLen, fb->buf, fb->len);
    buffer[headerLen + fb->len] = '
';
    buffer[headerLen + fb->len + 1] = '
';
    size_t outLen = headerLen + fb->len + 2;
    esp_camera_fb_return(fb);
    return outLen;
  });
  response->addHeader("Access-Control-Allow-Origin", "*");
  request->send(response);
}

/********** Web UI (with Voice) **********/
const char* INDEX_HTML = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>ESP32‑CAM Drone UI — Voice Enabled</title>
<style>
  body{font-family:system-ui;margin:0;padding:0;background:#0b0b0b;color:#fafafa}
  header{padding:12px 16px;background:#111;position:sticky;top:0;display:flex;gap:12px;align-items:center}
  button{padding:10px 14px;border-radius:12px;border:0;background:#222;color:#fff;cursor:pointer}
  button.primary{background:#2d6cdf}
  .row{display:flex;gap:12px;padding:12px;flex-wrap:wrap}
  .card{background:#141414;border-radius:16px;box-shadow:0 4px 20px rgba(0,0,0,.3);padding:12px}
  .video{max-width:100%;border-radius:12px}
  .pill{display:inline-block;padding:6px 10px;border-radius:999px;background:#222;margin-left:8px}
  .kv{display:grid;grid-template-columns:auto 1fr;gap:6px 12px}
  .mic{width:48px;height:48px;border-radius:50%;display:inline-flex;align-items:center;justify-content:center;font-weight:bold}
  .transcript{background:#0b0b0b;border:1px solid #222;padding:8px;border-radius:8px;margin-top:8px}
</style>
</head>
<body>
  <header>
    <strong>ESP32‑CAM Drone</strong>
    <span class="pill" id="net">...</span>
    <span class="pill" id="arm">Disarmed</span>
    <span class="pill" id="mode">LF: off, OA: on</span>
  </header>
  <div class="row">
    <div class="card" style="flex:2 1 380px">
      <img id="vid" class="video" src="/stream" />
    </div>
    <div class="card" style="flex:1 1 260px">
      <div class="kv">
        <div>Line error</div><div id="err">n/a</div>
        <div>ToF (m)</div><div id="tof">n/a</div>
      </div>
      <div style="margin-top:10px;display:flex;gap:8px;flex-wrap:wrap">
        <button class="primary" onclick="cmd('arm')">Arm</button>
        <button onclick="cmd('disarm')">Disarm</button>
        <button onclick="cmd('lf_on')">Line Follow ON</button>
        <button onclick="cmd('lf_off')">Line Follow OFF</button>
        <button onclick="cmd('oa_toggle')">Toggle Obstacle Avoid</button>
      </div>

      <div style="margin-top:12px">
        <div style="display:flex;align-items:center;gap:8px">
          <div id="micBtn" class="mic" style="background:#2d6cdf;color:#fff;cursor:pointer" onclick="toggleListening()">🎤</div>
          <div>Tap mic and speak a command (e.g. "arm", "disarm", "line follow on", "line follow off", "toggle obstacle")</div>
        </div>
        <div class="transcript" id="transcript">Transcript: —</div>
      </div>

    </div>
  </div>
<script>
 // WebSocket status
 const ws = new WebSocket(`ws://${location.host}/ws`);
 ws.onopen = ()=> document.getElementById('net').textContent='Online';
 ws.onclose = ()=> document.getElementById('net').textContent='Offline';
 ws.onmessage = (ev)=>{
   const s = JSON.parse(ev.data);
   document.getElementById('err').textContent = (s.line_err!==null)? s.line_err.toFixed(3):'n/a';
   document.getElementById('tof').textContent = (s.tof_m!==null)? s.tof_m.toFixed(2):'n/a';
   document.getElementById('arm').textContent = s.armed? 'Armed' : 'Disarmed';
   document.getElementById('mode').textContent = `LF: ${s.lf?'on':'off'}, OA: ${s.oa?'on':'off'}`;
 };
 function cmd(c){ fetch(`/api/${c}`).then(()=>{}); }

 // Voice recognition using Web Speech API
 let recognizing = false;
 let recognition;
 if ('webkitSpeechRecognition' in window || 'SpeechRecognition' in window) {
   const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
   recognition = new SR();
   recognition.lang = 'en-US';
   recognition.interimResults = false;
   recognition.maxAlternatives = 1;

   recognition.onstart = ()=> { recognizing = true; document.getElementById('micBtn').style.background = '#e33'; };
   recognition.onend = ()=> { recognizing = false; document.getElementById('micBtn').style.background = '#2d6cdf'; };
   recognition.onresult = (event)=>{
     const text = event.results[0][0].transcript.trim();
     document.getElementById('transcript').textContent = 'Transcript: ' + text;
     // send to ESP32 voice endpoint for parsing/action
     fetch(`/api/voice?text=${encodeURIComponent(text)}`).then(()=>{});
   };
   recognition.onerror = (e)=>{ console.warn('Speech error', e); };
 }

 function toggleListening(){
   if (!recognition) { alert('Voice not supported in this browser. Use Chrome/Edge on Android or desktop.'); return; }
   if (recognizing) { recognition.stop(); } else { recognition.start(); }
 }
</script>
</body>
</html>
)HTML";

void ws_broadcast_state() {
  char buf[192];
  const char *err = isnan(g_state.line_err)? "null" : String(g_state.line_err,3).c_str();
  const char *tof = isnan(g_state.tof_m)? "null" : String(g_state.tof_m,2).c_str();
  snprintf(buf, sizeof(buf), "{\"line_err\":%s,\"tof_m\":%s,\"armed\":%s,\"lf\":%s,\"oa\":%s}",
           err, tof, g_state.armed?"true":"false", g_line_follow_enabled?"true":"false", g_obstacle_avoid_enabled?"true":"false");
  ws.textAll(buf);
}

/********** Frame processing task **********/
uint32_t last_process_us = 0;

void process_frame_and_control() {
  const uint32_t now = micros32();
  if (now - last_process_us < 100000) return; // 10Hz processing
  last_process_us = now;

  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) return;

  uint8_t* rgb = (uint8_t*)malloc(fb->width * fb->height * 3);
  if (!rgb) { esp_camera_fb_return(fb); return; }
  bool ok = fmt2rgb888(fb->buf, fb->len, fb->format, rgb);
  int w = fb->width, h = fb->height;
  esp_camera_fb_return(fb);
  if (!ok) { free(rgb); return; }

  float err = compute_line_error_from_rgb888(rgb, w, h);
  free(rgb);
  g_state.line_err = err;

  if (TOF_AVAILABLE) {
    #if __has_include(<Adafruit_VL53L0X.h>)
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest(&measure, false);
      if (measure.RangeStatus == 0) {
        g_state.tof_m = measure.RangeMilliMeter / 1000.0f;
      } else {
        g_state.tof_m = NAN;
      }
    #endif
  }

  uint16_t roll = RC_MID, pitch = RC_MID, thr = g_base_throttle, yaw = RC_MID;

  if (g_line_follow_enabled && !isnan(err)) {
    const float Kp = 250.0f; // gain -> converts [-1..1] to RC delta; tune
    int delta = (int)(-Kp * err); // negative: line right -> roll right
    roll = constrain(RC_MID + delta, RC_MIN, RC_MAX);
  }

  if (g_obstacle_avoid_enabled && !isnan(g_state.tof_m) && g_state.tof_m < OBSTACLE_M) {
    thr = RC_MID; // reduce
    pitch = RC_MID; // stop forward
  }

  if (g_armed) {
    mav_rc_override(roll, pitch, thr, yaw);
  }
}

/********** Voice command parsing (server-side) **********/
void handle_voice_text(const String &txt) {
  String s = txt;
  s.toLowerCase();
  s.trim();
  if (s.indexOf("arm") >= 0 && s.indexOf("dis") < 0) {
    g_armed = true; mav_arm(true);
  } else if (s.indexOf("disarm") >= 0 || (s.indexOf("arm")>=0 && s.indexOf("not")>=0)) {
    g_armed = false; mav_arm(false);
  } else if (s.indexOf("line") >= 0 && s.indexOf("on") >= 0) {
    g_line_follow_enabled = true;
  } else if (s.indexOf("line") >= 0 && s.indexOf("off") >= 0) {
    g_line_follow_enabled = false;
  } else if (s.indexOf("toggle") >= 0 && s.indexOf("obstacle") >= 0) {
    g_obstacle_avoid_enabled = !g_obstacle_avoid_enabled;
  } else if (s.indexOf("stop") >= 0 || s.indexOf("hold") >= 0) {
    // stop motion: disable LF and set throttle to mid
    g_line_follow_enabled = false;
    g_base_throttle = RC_MID;
  } else if (s.indexOf("throttle up") >= 0 || s.indexOf("increase throttle") >= 0) {
    g_base_throttle = min((int)g_base_throttle + 50, RC_MAX);
  } else if (s.indexOf("throttle down") >= 0 || s.indexOf("decrease throttle") >= 0) {
    g_base_throttle = max((int)g_base_throttle - 50, RC_MIN);
  }
}

/********** HTTP handlers **********/
void setup_http() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
    AsyncWebServerResponse *res = req->beginResponse(200, "text/html", INDEX_HTML);
    res->addHeader("Access-Control-Allow-Origin", "*");
    req->send(res);
  });
  server.on("/stream", HTTP_GET, handle_stream);

  server.on("/api/arm", HTTP_GET, [](AsyncWebServerRequest* req){ g_armed = true; mav_arm(true); req->send(200); });
  server.on("/api/disarm", HTTP_GET, [](AsyncWebServerRequest* req){ g_armed = false; mav_arm(false); req->send(200); });
  server.on("/api/lf_on", HTTP_GET, [](AsyncWebServerRequest* req){ g_line_follow_enabled = true; req->send(200); });
  server.on("/api/lf_off", HTTP_GET, [](AsyncWebServerRequest* req){ g_line_follow_enabled = false; req->send(200); });
  server.on("/api/oa_toggle", HTTP_GET, [](AsyncWebServerRequest* req){ g_obstacle_avoid_enabled = !g_obstacle_avoid_enabled; req->send(200); });

  // Voice endpoint: called by browser Web Speech API with recognized text
  server.on("/api/voice", HTTP_GET, [](AsyncWebServerRequest* req){
    if (req->hasParam("text")) {
      String t = req->getParam("text")->value();
      handle_voice_text(t);
    }
    req->send(200);
  });

  ws.onEvent([](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len){
    if (type == WS_EVT_CONNECT) { ws_broadcast_state(); }
  });
  server.addHandler(&ws);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();
}

/********** Setup/Loop **********/
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // brownout off
  Serial.begin(115200);
  UART_PORT.begin(BAUD_TELEM, SERIAL_8N1, PIN_RX2, PIN_TX2);

  if (!init_camera()) {
    Serial.println("Camera init failed");
  } else {
    camera_init_ok = true;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.printf("AP IP: %s
", WiFi.softAPIP().toString().c_str());

  #if __has_include(<Adafruit_VL53L0X.h>)
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!lox.begin()) { TOF_AVAILABLE = false; }
  #endif

  setup_http();
}

uint32_t last_hb = 0, last_ws = 0;

void loop() {
  uint32_t now = millis();
  if (now - last_hb > 1000) { mav_send_heartbeat(); last_hb = now; }
  if (now - last_ws > 200) { ws_broadcast_state(); last_ws = now; }

  process_frame_and_control();
}
