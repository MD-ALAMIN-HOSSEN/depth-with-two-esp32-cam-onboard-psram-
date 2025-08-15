#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include "esp_camera.h"

#include "esp_heap_caps.h"

// CAMERA MODEL
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
//#include "esp32/spiram.h" 

#define SRC_WIDTH  160
#define SRC_HEIGHT 120
#define IMG_WIDTH   160
#define IMG_HEIGHT  120

#define HALF_HEIGHT (IMG_HEIGHT / 2)

const char* ssid = "ESP32-CAM-AP";
const char* password = "12345678";
IPAddress clientIP(192,168,4,2);


// Store in PSRAM
uint8_t* apBottomHalf;
uint8_t* clientTopHalf;
uint8_t* apTopHalf;
uint8_t* sampledFrame;

uint8_t* worldXmap;
uint8_t* worldYmap;
float*   worldZmap;
// uint8_t PSRAM_ATTR clientTopHalf[IMG_WIDTH * HALF_HEIGHT];
// uint8_t PSRAM_ATTR apTopHalf[IMG_WIDTH * HALF_HEIGHT];
// uint8_t PSRAM_ATTR apBottomHalf[IMG_WIDTH * HALF_HEIGHT];

// uint8_t PSRAM_ATTR sampledFrame[IMG_WIDTH * IMG_HEIGHT];

// uint8_t PSRAM_ATTR worldXmap[HALF_HEIGHT * IMG_WIDTH];
// uint8_t PSRAM_ATTR worldYmap[HALF_HEIGHT * IMG_WIDTH];
// float   PSRAM_ATTR worldZmap[HALF_HEIGHT * IMG_WIDTH];

unsigned long lastDepthTime = 0;
int halfBlock = 3; // 3-> 7x7, 2->5x5, 1->3x3

WebServer server(80);
WiFiClient client;



void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QQVGA; // 160x120
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (1);
  }
}


void fetchClientTopHalf() {
  HTTPClient http;
  http.begin(client, "http://192.168.4.2/send_client_top_half");  // client IP must be correct
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String body = http.getString();

    int idx = 0, start = 0;
    while (idx < HALF_HEIGHT * IMG_WIDTH && start < body.length()) {
      int end = body.indexOf(',', start);
      if (end == -1) end = body.length();
      clientTopHalf[idx++] = body.substring(start, end).toInt();
      start = end + 1;
    }

    Serial.println("✅ Top half received from client.");
  } else {
    Serial.printf("❌ Failed to fetch client top half. HTTP code: %d\n", httpCode);
  }

  http.end();
}

void printClientTopHalf() {
  Serial.println("==== clientTopHalf (160x60) ====");  // Adjust if your IMG_WIDTH is not 80

  for (int y = 0; y < HALF_HEIGHT; y++) {
    for (int x = 0; x < IMG_WIDTH; x++) {
      int index = y * IMG_WIDTH + x;
      Serial.print(clientTopHalf[index]);
      Serial.print("\t");
    }
    Serial.println();
  }

  Serial.println("=================================");
}


void gaussianBlurSameSize(uint8_t* src, uint8_t* dest) {
  for (int y = 0; y < SRC_HEIGHT; y++) {
    for (int x = 0; x < SRC_WIDTH; x++) {

      // Edge handling — fallback to original pixel
      if (x <= 0 || y <= 0 || x >= SRC_WIDTH - 1 || y >= SRC_HEIGHT - 1) {
        dest[y * SRC_WIDTH + x] = src[y * SRC_WIDTH + x];
        continue;
      }

      // 3×3 Gaussian kernel: [1 2 1; 2 4 2; 1 2 1], sum = 16
      uint16_t sum =
        1 * src[(y - 1) * SRC_WIDTH + (x - 1)] +
        2 * src[(y - 1) * SRC_WIDTH + (x    )] +
        1 * src[(y - 1) * SRC_WIDTH + (x + 1)] +

        2 * src[(y    ) * SRC_WIDTH + (x - 1)] +
        4 * src[(y    ) * SRC_WIDTH + (x    )] +
        2 * src[(y    ) * SRC_WIDTH + (x + 1)] +

        1 * src[(y + 1) * SRC_WIDTH + (x - 1)] +
        2 * src[(y + 1) * SRC_WIDTH + (x    )] +
        1 * src[(y + 1) * SRC_WIDTH + (x + 1)];

      dest[y * SRC_WIDTH + x] = sum / 16;
    }
  }
}


void captureAndSplit() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    return;
  }
  gaussianBlurSameSize(fb->buf, sampledFrame);  //downsampledFrame
  esp_camera_fb_return(fb);

  for (int y = 0; y < HALF_HEIGHT; y++) {
    memcpy(&apTopHalf[y * IMG_WIDTH], &sampledFrame[y * IMG_WIDTH], IMG_WIDTH);  // Top half
    memcpy(&apBottomHalf[y * IMG_WIDTH], &sampledFrame[(y + HALF_HEIGHT) * IMG_WIDTH], IMG_WIDTH);  // Bottom half
  }
}

int findBestHorizontalPixalDisparityTop(int x, int y, int maxDisparity, int halfBlock) {
  int bestDisparity = 0;
  int minSAD = INT32_MAX;

  int maxAllowedD = IMG_WIDTH - 1 - (x + halfBlock);
  if (maxAllowedD < 0) return 0;
  if (maxDisparity > maxAllowedD) maxDisparity = maxAllowedD;

  for (int d = 0; d <= maxDisparity; d++) {
    int sad = 0;
    bool stopEarly = false;

    for (int dy = -halfBlock; dy <= halfBlock; dy++) {
      int apY = y + dy;
      if (apY < 0 || apY >= HALF_HEIGHT) { stopEarly = true; break; }
      int rowBase = apY * IMG_WIDTH;

      for (int dx = -halfBlock; dx <= halfBlock; dx++) {
        int apX = x + dx;
        int clientX = x + dx + d;
        if (apX < 0 || apX >= IMG_WIDTH || clientX < 0 || clientX >= IMG_WIDTH) {
          stopEarly = true; break;
        }
        sad += abs(apTopHalf[rowBase + apX] - clientTopHalf[rowBase + clientX]);
        if (sad >= minSAD) { stopEarly = true; break; }
      }
      if (stopEarly) break;
    }

    if (!stopEarly && sad < minSAD) {
      minSAD = sad;
      bestDisparity = d;
    }
  }
  return bestDisparity;
}

float computeDepth(int disparity, float focalLength, float baseline) {
    if (disparity == 0) return 9999.0f;  // max depth cap // avoid division by zero, assume far away

    return (focalLength * baseline) / disparity;
}

void computeTopDepth() {
  float focalLength = 50.0f;  // Adjusted due to 2x downscale
  float baseline = 10.0f;     // cm
  float cx = IMG_WIDTH / 2.0f;       // 160 / 2 = 80
  float cy = HALF_HEIGHT / 2.0f;     // 60 / 2 = 30
  int index = 0;

  for (int y = 2; y < HALF_HEIGHT - 2; y++) {       // y = 2 to 27
    //calculateRowSums(y);

    for (int x = 2; x < IMG_WIDTH - 2; x++) {        // x = 2 to 77
      int disp = findBestHorizontalPixalDisparityTop(x, y, 12, halfBlock);  // e.g. maxDisparity = 57   IMG_WIDTH - x - 2
      float depth = computeDepth(disp, focalLength, baseline);

      if (depth > 255.0f || disp == 0) depth = 255.0f;

      float worldX = (x - cx) * depth / focalLength;
      float worldY = (y - cy) * depth / focalLength;

      int scaledX = (int)(worldX + 127.5f);
      int scaledY = (int)(worldY + 127.5f);

      scaledX = constrain(scaledX, 0, 255);
      scaledY = constrain(scaledY, 0, 255);

      worldXmap[index] = (uint8_t)scaledX;
      worldYmap[index] = (uint8_t)scaledY;
      worldZmap[index] = depth;
      index++;
    }
  }
}


void handleGetDepth() {
  Serial.println("Start depth flow...");
  captureAndSplit();
  // Trigger client capture
  HTTPClient http;
  http.begin(client, "http://192.168.4.2/capture");
  http.GET();
  http.end();

  delay(100);////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Send BOTTOM half to client
  HTTPClient http2;
  http2.begin(client, "http://192.168.4.2/receive_bottom_half");  // change endpoint if needed
  
  String payload = "";
  for (int i = 0; i < HALF_HEIGHT * IMG_WIDTH; i++) {
    payload += String(apBottomHalf[i]);
    if (i < HALF_HEIGHT * IMG_WIDTH - 1) payload += ",";
  }
  http2.POST(payload);
  http2.end();//////////////////////////////////////////
  //get the right half
  //Fetch clientRightHalf from client
  fetchClientTopHalf();
  //fetchClientRightHalf();
  printClientTopHalf();
  //calculate x, y, z values and store
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //computeRightDepth();
  unsigned long startTime = millis();
  computeTopDepth();
  lastDepthTime = millis() - startTime;

  String json = "{";
  
  json += "\"X\":[";
  for (int i = 0; i < HALF_HEIGHT * IMG_WIDTH; i++) {
    json += worldXmap[i];
    if (i < HALF_HEIGHT * IMG_WIDTH - 1) json += ",";
  }
  json += "],\"Y\":[";
  for (int i = 0; i < HALF_HEIGHT * IMG_WIDTH; i++) {
    json += worldYmap[i];
    if (i < HALF_HEIGHT * IMG_WIDTH - 1) json += ",";
  }
  json += "],\"Z\":[";
  for (int i = 0; i < HALF_HEIGHT * IMG_WIDTH; i++) {
    json += String(worldZmap[i], 2);
    if (i < HALF_HEIGHT * IMG_WIDTH - 1) json += ",";
  }
  json += "]}";

  server.send(200, "application/json", json);

}


void handleGetDepthTime() {
  String json = "{\"elapsed_ms\":" + String(lastDepthTime) + "}";
  server.send(200, "application/json", json);
}


void setup() {
  Serial.begin(115200);


  apBottomHalf     = (uint8_t*) heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT, MALLOC_CAP_SPIRAM);
  clientTopHalf    = (uint8_t*) heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT, MALLOC_CAP_SPIRAM);
  apTopHalf = (uint8_t*) heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT, MALLOC_CAP_SPIRAM);
  sampledFrame     = (uint8_t*) heap_caps_malloc(IMG_WIDTH * IMG_HEIGHT, MALLOC_CAP_SPIRAM);

  worldXmap = (uint8_t*) heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT, MALLOC_CAP_SPIRAM);
  worldYmap = (uint8_t*) heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT, MALLOC_CAP_SPIRAM);
  worldZmap = (float*)   heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT * sizeof(float), MALLOC_CAP_SPIRAM);

  if(!apBottomHalf || !clientTopHalf || !apTopHalf || !sampledFrame ||
     !worldXmap || !worldYmap || !worldZmap) {
    Serial.println("PSRAM allocation failed!");

     }
  if (psramFound()) {
    Serial.printf("PSRAM found! Size: %u bytes (%.2f KB)\n", ESP.getPsramSize(), ESP.getPsramSize() / 1024.0);
  } else {
    Serial.println("PSRAM not found!");
  }

 // WiFi.onEvent(WiFiEvent);  // Register event handler
  setupCamera();
  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  server.on("/get_depth", HTTP_GET, handleGetDepth);
  server.on("/get_depth_time_ap", HTTP_GET, handleGetDepthTime);
  server.begin();
  Serial.println("AP ready");
}

void loop() {
  server.handleClient();

}