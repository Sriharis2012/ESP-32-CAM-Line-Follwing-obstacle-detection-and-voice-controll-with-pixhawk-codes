#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "mavlink/common/mavlink.h"

#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

const char* ssid = "ESP32CAM_Drone";
const char* password = "esp32pixhawk";

HardwareSerial PixSerial(2);
#define RXD2 16
#define TXD2 17

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool tof_ready = false;

WebServer server(80);

bool g_armed = false;
bool g_line_follow_enabled = false;
bool g_obstacle_enabled = false;
int g_base_throttle = 1400;

const char* INDEX_HTML PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>ESP32 Drone</title>
<style>
body { font-family: Arial; text-align:center; background:#111; color:white;}
button { padding:12px; margin:6px; border-radius:12px; font-size:18px;}
#video { width:100%; max-width:480px; border-radius:12px;}
</style>
</head>
<body>
<h2>ESP32-CAM Drone</h2>
<img id="video" src="/stream">
<br>
<button onclick="sendCmd('arm')">Arm</button>
<button onclick="sendCmd('disarm')">Disarm</button>
<button onclick="sendCmd('line_on')">Line Follow ON</button>
<button onclick="sendCmd('line_off')">Line Follow OFF</button>
<button onclick="sendCmd('toggle_obstacle')">Toggle Obstacle</button>
<br>
<button onclick="sendCmd('stop')">STOP</button>
<button onclick="sendCmd('throttle_up')">Throttle+</button>
<button onclick="sendCmd('throttle_down')">Throttle-</button>
<br><br>
<button id="micBtn">🎤 Voice</button>
<p id="voiceStatus">Voice: idle</p>
<script>
function sendCmd(cmd){ fetch('/api/'+cmd); }
let recognition;
if('webkitSpeechRecognition' in window){
 recognition=new webkitSpeechRecognition();
 recognition.continuous=false;
 recognition.lang="en-US";
 recognition.onresult=(e)=>{
   let t=e.results[0][0].transcript.toLowerCase();
   document.getElementById("voiceStatus").innerText="Heard: "+t;
   fetch('/api/voice?cmd='+t);
 };
}
document.getElementById("micBtn").onclick=()=>{
 if(recognition){ recognition.start(); document.getElementById("voiceStatus").innerText="Listening..."; }
};
</script>
</body>
</html>
)rawliteral";

void sendRcOverride(int roll, int pitch, int throttle, int yaw) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_rc_channels_override_pack(
      1, 200, &msg, 1, 0, roll, pitch, throttle, yaw, 0, 0, 0, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  PixSerial.write(buf, len);
}

void handleJpgStream(void) {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);
  while (client.connected()) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) continue;
    server.sendContent("--frame\r\nContent-Type: image/jpeg\r\n\r\n");
    client.write(fb->buf, fb->len);
    server.sendContent("\r\n");
    esp_camera_fb_return(fb);
  }
}

void lineFollow() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) return;

  int leftSum=0, rightSum=0;
  int width = 160; // scaled down
  int step = fb->len / width;
  for (int i=0; i<width; i++) {
    uint8_t val = fb->buf[i*step];
    if (i < width/2) leftSum += val;
    else rightSum += val;
  }
  esp_camera_fb_return(fb);

  int error = leftSum - rightSum;
  int roll = 1500 + error/50;
  int throttle = g_base_throttle;
  sendRcOverride(roll, 1500, throttle, 1500);
}

void checkObstacle() {
  if (!tof_ready) return;
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeMilliMeter < 800 && measure.RangeStatus == 0) {
    sendRcOverride(1500, 1500, 1200, 1500); // stop/hover
  }
}

void handleRoot() { server.send(200, "text/html", INDEX_HTML); }
void handleVoice() {
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd"); cmd.toLowerCase();
    if (cmd.indexOf("arm")>=0) g_armed=true;
    if (cmd.indexOf("disarm")>=0) g_armed=false;
    if (cmd.indexOf("line follow on")>=0) g_line_follow_enabled=true;
    if (cmd.indexOf("line follow off")>=0) g_line_follow_enabled=false;
    if (cmd.indexOf("toggle obstacle")>=0) g_obstacle_enabled=!g_obstacle_enabled;
    if (cmd.indexOf("stop")>=0) g_line_follow_enabled=false;
    if (cmd.indexOf("throttle up")>=0) g_base_throttle+=50;
    if (cmd.indexOf("throttle down")>=0) g_base_throttle-=50;
  }
  server.send(200,"text/plain","OK");
}

void startCameraServer() {
  server.on("/", handleRoot);
  server.on("/stream", HTTP_GET, [](){ handleJpgStream(); });
  server.on("/api/arm", [](){ g_armed=true; server.send(200,"text/plain","ARM"); });
  server.on("/api/disarm", [](){ g_armed=false; server.send(200,"text/plain","DISARM"); });
  server.on("/api/line_on", [](){ g_line_follow_enabled=true; server.send(200,"text/plain","LINE_ON"); });
  server.on("/api/line_off", [](){ g_line_follow_enabled=false; server.send(200,"text/plain","LINE_OFF"); });
  server.on("/api/toggle_obstacle", [](){ g_obstacle_enabled=!g_obstacle_enabled; server.send(200,"text/plain","OBST"); });
  server.on("/api/stop", [](){ g_line_follow_enabled=false; server.send(200,"text/plain","STOP"); });
  server.on("/api/throttle_up", [](){ g_base_throttle+=50; server.send(200,"text/plain","THR+"); });
  server.on("/api/throttle_down", [](){ g_base_throttle-=50; server.send(200,"text/plain","THR-"); });
  server.on("/api/voice", handleVoice);
  server.begin();
}

void setup() {
  Serial.begin(115200);
  PixSerial.begin(57600, SERIAL_8N1, RXD2, TXD2);

  WiFi.softAP(ssid, password);

  camera_config_t config;
  config.ledc_channel = 0;
  config.ledc_timer = 0;
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
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  if (lox.begin()) {
    tof_ready = true;
    Serial.println("TOF ready");
  }

  startCameraServer();
}

void loop() {
  server.handleClient();

  if (g_armed) {
    if (g_line_follow_enabled) {
      lineFollow();
    } else {
      sendRcOverride(1500, 1500, g_base_throttle, 1500); // hover
    }

    if (g_obstacle_enabled) {
      checkObstacle();
    }
  }
}
