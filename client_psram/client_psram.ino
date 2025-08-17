#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include "esp_camera.h"


// CAMERA MODEL
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
//#include "esp32/spiram.h" 

#define SRC_WIDTH  160
#define SRC_HEIGHT 120
#define IMG_WIDTH   160
#define IMG_HEIGHT  120

#define HALF_HEIGHT (IMG_HEIGHT / 2)

// NETWORK
const char* ssid = "ESP32-CAM-AP";
const char* password = "12345678";
IPAddress apIP(192,168,4,1);

uint8_t* apBottomHalf;
uint8_t* clientTopHalf;
uint8_t* clientBottomHalf;
uint8_t* sampledFrame;

uint8_t* worldXmap;
uint8_t* worldYmap;
float*   worldZmap;
// uint8_t PSRAM_ATTR apBottomHalf[IMG_WIDTH * HALF_HEIGHT];///////////
// uint8_t PSRAM_ATTR clientTopHalf[IMG_WIDTH * HALF_HEIGHT];
// uint8_t PSRAM_ATTR clientBottomHalf[IMG_WIDTH * HALF_HEIGHT];

// uint8_t PSRAM_ATTR sampledFrame[IMG_WIDTH * IMG_HEIGHT];

// uint8_t PSRAM_ATTR worldXmap[HALF_HEIGHT * IMG_WIDTH];
// uint8_t PSRAM_ATTR worldYmap[HALF_HEIGHT * IMG_WIDTH];
// float   PSRAM_ATTR worldZmap[HALF_HEIGHT * IMG_WIDTH];

unsigned long lastDepthTime = 0;
int halfBlock = 3; // 3-> 7x7, 2->5x5, 1->3x3

WebServer server(80);

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
  config.frame_size   = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (1);
  }
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
  gaussianBlurSameSize(fb->buf, sampledFrame);/////////////
  esp_camera_fb_return(fb);

  for (int y = 0; y < HALF_HEIGHT; y++) {
    memcpy(&clientTopHalf[y * IMG_WIDTH], &sampledFrame[y * IMG_WIDTH], IMG_WIDTH);  // Top half
    memcpy(&clientBottomHalf[y * IMG_WIDTH], &sampledFrame[(y + HALF_HEIGHT) * IMG_WIDTH], IMG_WIDTH);  // Bottom half
  }
}

void handleCapture() {
  captureAndSplit();
  server.send(200, "text/plain", "Client capture OK");
}

//receved bottom half
void handleReceiveApBottomHalf() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    int idx = 0, start = 0;
    while (idx < HALF_HEIGHT * IMG_WIDTH) {
      int end = body.indexOf(',', start);
      if (end == -1) end = body.length();
      apBottomHalf[idx] = body.substring(start, end).toInt();////////////////////
      idx++;
      start = end + 1;
    }

    ////////////////
    
    printApBottomTopHalf();
    //then go to calculation
    unsigned long startTime = millis();
    computeBottomDepth();////////////////////////////
    lastDepthTime = millis() - startTime;

    server.send(200, "text/plain", "Left depth sent");
  } else {
    server.send(400, "text/plain", "No data");
  }
}

//send top half
void handleSendClientTopHalf() {
  String payload = "";
  for (int i = 0; i < HALF_HEIGHT * IMG_WIDTH; i++) {
    payload += String(clientTopHalf[i]);
    if (i < HALF_HEIGHT * IMG_WIDTH - 1) payload += ",";
  }
  server.send(200, "text/plain", payload);
}

void printApBottomTopHalf() {
  Serial.println("==== apBottomHalf (160x60) ====");

  for (int y = 0; y < HALF_HEIGHT; y++) {
    for (int x = 0; x < IMG_WIDTH; x++) {
      int index = y * IMG_WIDTH + x;
      Serial.print(apBottomHalf[index]);
      Serial.print("\t"); // Tab-separated for readability
    }
    Serial.println(); // Newline after each row
  }

  Serial.println("=================================");
}

void handleGetXYZclient() {
    // calculate max size needed
    size_t bufSize = HALF_HEIGHT * IMG_WIDTH * 12 * 3 + 256; // rough estimate
    char* jsonBuf = (char*) heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
    if (!jsonBuf) {
        server.send(500, "text/plain", "Failed to allocate PSRAM");
        return;
    }
    
    char* ptr = jsonBuf;
    ptr += sprintf(ptr, "{");

    // X array
    ptr += sprintf(ptr, "\"X\":[");
    for (int i = 0; i < HALF_HEIGHT * IMG_WIDTH; i++) {
        ptr += sprintf(ptr, "%d", (int)worldXmap[i]);
        if (i < HALF_HEIGHT * IMG_WIDTH - 1) ptr += sprintf(ptr, ",");
    }
    ptr += sprintf(ptr, "],");

    // Y array
    ptr += sprintf(ptr, "\"Y\":[");
    for (int i = 0; i < HALF_HEIGHT * IMG_WIDTH; i++) {
        ptr += sprintf(ptr, "%d", (int)worldYmap[i]);
        if (i < HALF_HEIGHT * IMG_WIDTH - 1) ptr += sprintf(ptr, ",");
    }
    ptr += sprintf(ptr, "],");

    // Z array
    ptr += sprintf(ptr, "\"Z\":[");
    for (int i = 0; i < HALF_HEIGHT * IMG_WIDTH; i++) {
        ptr += sprintf(ptr, "%.2f", worldZmap[i]);
        if (i < HALF_HEIGHT * IMG_WIDTH - 1) ptr += sprintf(ptr, ",");
    }
    ptr += sprintf(ptr, "]}");

    server.send(200, "application/json", jsonBuf);
    free(jsonBuf); // free PSRAM
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
        sad += abs(apBottomHalf[rowBase + apX] - clientBottomHalf[rowBase + clientX]);
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

void computeBottomDepth() {
  float focalLength = 211.0f;  
  float baseline = 10.0f;     // cm
  float cx = IMG_WIDTH / 2.0f;       // 160 / 2 = 80
  float cy = HALF_HEIGHT / 2.0f;     // 60 / 2 = 30
  int index = 0;

  for (int y = 3; y < HALF_HEIGHT - 3; y++) {       // y = 2 to 27
    //calculateRowSums(y);

    for (int x = 3; x < IMG_WIDTH - 3; x++) {        // x = 2 to 77
      //int disp = findBestHorizontalDisparityBottom(x, y, IMG_WIDTH - x - 2);  // e.g. maxDisparity = 57
      int disp = findBestHorizontalPixalDisparityTop(x, y, 32, halfBlock); 
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


void handleGetDepthTime() {
  String json = "{\"elapsed_ms\":" + String(lastDepthTime) + "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);

  apBottomHalf     = (uint8_t*) heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT, MALLOC_CAP_SPIRAM);
  clientTopHalf    = (uint8_t*) heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT, MALLOC_CAP_SPIRAM);
  clientBottomHalf = (uint8_t*) heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT, MALLOC_CAP_SPIRAM);
  sampledFrame     = (uint8_t*) heap_caps_malloc(IMG_WIDTH * IMG_HEIGHT, MALLOC_CAP_SPIRAM);

  worldXmap = (uint8_t*) heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT, MALLOC_CAP_SPIRAM);
  worldYmap = (uint8_t*) heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT, MALLOC_CAP_SPIRAM);
  worldZmap = (float*)   heap_caps_malloc(IMG_WIDTH * HALF_HEIGHT * sizeof(float), MALLOC_CAP_SPIRAM);

  if(!apBottomHalf || !clientTopHalf || !clientBottomHalf || !sampledFrame ||
     !worldXmap || !worldYmap || !worldZmap) {
    Serial.println("PSRAM allocation failed!");

     }
    if (psramFound()) {
    Serial.printf("PSRAM found! Size: %u bytes (%.2f KB)\n", ESP.getPsramSize(), ESP.getPsramSize() / 1024.0);
  } else {
    Serial.println("PSRAM not found!");
  }

  setupCamera();

  Serial.println("🔌 Connecting to AP...");
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) { // max 15 seconds
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Client connected to AP!");
    Serial.print("📶 IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ Failed to connect to AP. Check if AP is powered and broadcasting.");
    return;
  }

  // Setup web routes
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/receive_bottom_half", HTTP_POST, handleReceiveApBottomHalf);
  server.on("/send_client_top_half", HTTP_GET, handleSendClientTopHalf);

  server.on("/get_xyz_client", HTTP_GET, handleGetXYZclient);// to get depth from clent after calculation
   server.on("/get_depth_time_client", HTTP_GET, handleGetDepthTime);

  server.begin();
  Serial.println("🌐 Client server started");
}


void loop() {
  server.handleClient();
  
}