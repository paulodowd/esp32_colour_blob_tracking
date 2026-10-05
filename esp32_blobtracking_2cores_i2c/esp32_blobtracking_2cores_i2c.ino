/*
  AI Thinker ESP32-CAM (OV2640)
  Dual-core pipeline:
    - Core 0: capture_task()  -> esp_camera_fb_get() -> queue
    - Core 1: process_task()  -> blob detect -> publish over I2C -> esp_camera_fb_return()

  I2C slave output (fixed 20-byte packet):
    uint32_t seq
    uint8_t  valid
    uint8_t  reserved[3]
    float    cx
    float    cy
    float    bw   (bbox width in pixels)
    float    bh   (bbox height in pixels)

  I2C:
    SDA = GPIO14
    SCL = GPIO15
    Address = 0x42
    Freq = 100 kHz

  Notes:
  - ESP32 is 3.3V only. If master is 5V, use a level shifter or 3.3V pullups.
  - RGB565 byte order may need swapping. Toggle RGB565_BYTE_SWAP if color classification is wrong.
*/

#include <Arduino.h>
#include <Wire.h>
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ===================== Camera pins: AI Thinker ESP32-CAM =====================
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

// ===================== Blob detection =====================
struct BlobResult {
  bool found;
  uint32_t count;
  float cx, cy;
  int xmin, ymin, xmax, ymax;
};

// ===================== I2C slave config =====================
static constexpr uint8_t  I2C_SLAVE_ADDR = 0x42;
static constexpr int      I2C_SDA_PIN    = 14;
static constexpr int      I2C_SCL_PIN    = 15;
static constexpr uint32_t I2C_FREQ_HZ    = 100000;

// ===================== RGB565 byte order =====================
#define RGB565_BYTE_SWAP 1
static inline uint16_t bswap16(uint16_t v) { return (uint16_t)((v << 8) | (v >> 8)); }
static inline uint16_t rgb565_read(uint16_t p) {
#if RGB565_BYTE_SWAP
  return bswap16(p);
#else
  return p;
#endif
}



// RGB565 -> 8-bit-ish channels
static inline uint8_t r8_from_rgb565(uint16_t px) { return ((px >> 11) & 0x1F) * 255 / 31; }
static inline uint8_t g8_from_rgb565(uint16_t px) { return ((px >> 5)  & 0x3F) * 255 / 63; }
static inline uint8_t b8_from_rgb565(uint16_t px) { return ( px        & 0x1F) * 255 / 31; }



// Tunables
static constexpr uint8_t  RED_MIN       = 80;    // 40..140
static constexpr float    DOMINANCE     = 1.4f;  // 1.1..1.8
static constexpr int      RED_MARGIN    = 50;    // 20..90
static constexpr int      STRIDE        = 2;     // 1..4

// MIN_COUNT depends on frame size and stride. Start with these:
static constexpr uint32_t MIN_COUNT_QQVGA_STR2 = 80;   // 160x120, stride=2
static constexpr uint32_t MIN_COUNT_QVGA_STR2  = 300;  // 320x240, stride=2 (approx 4x area)

// Will be set after camera init:
static uint32_t g_min_count = MIN_COUNT_QQVGA_STR2;

static inline bool is_red_pixel(uint8_t r, uint8_t g, uint8_t b)
{
  if (r < RED_MIN) return false;
  if (r < (uint8_t)(g * DOMINANCE)) return false;
  if (r < (uint8_t)(b * DOMINANCE)) return false;

  uint8_t maxgb = (g > b) ? g : b;
  if ((int)r - (int)maxgb < RED_MARGIN) return false;

  return true;
}

static BlobResult find_red_blob_rgb565(const camera_fb_t *fb, int stride)
{
  BlobResult res{};
  res.found = false;
  res.count = 0;
  res.cx = res.cy = 0.0f;
  res.xmin = res.ymin =  1e9;
  res.xmax = res.ymax = -1e9;

  if (!fb || fb->format != PIXFORMAT_RGB565) return res;

  const int w = fb->width;
  const int h = fb->height;

  const uint16_t *pix = reinterpret_cast<const uint16_t*>(fb->buf);

  uint64_t sumx = 0, sumy = 0;
  uint32_t count = 0;

  for (int y = 0; y < h; y += stride) {
    int row = y * w;
    for (int x = 0; x < w; x += stride) {
      uint16_t p = rgb565_read(pix[row + x]);

      uint8_t r = r8_from_rgb565(p);
      uint8_t g = g8_from_rgb565(p);
      uint8_t b = b8_from_rgb565(p);

      if (is_red_pixel(r, g, b)) {
        count++;
        sumx += x;
        sumy += y;
        if (x < res.xmin) res.xmin = x;
        if (y < res.ymin) res.ymin = y;
        if (x > res.xmax) res.xmax = x;
        if (y > res.ymax) res.ymax = y;
      }
    }
  }

  if (count >= g_min_count) {
    res.found = true;
    res.count = count;
    res.cx = (float)sumx / (float)count;
    res.cy = (float)sumy / (float)count;
  }

  return res;
}

// ===================== I2C packet =====================
#pragma pack(push, 1)
struct BlobPacket {
  uint32_t seq;
  uint8_t  valid;
  float cx;
  float cy;
  float bw;
  float bh;
};
#pragma pack(pop)
static_assert(sizeof(BlobPacket) == 21, "BlobPacket must be 21 bytes");

static BlobPacket g_pkt = {0,0,0,0,0,0};
static portMUX_TYPE g_pktMux = portMUX_INITIALIZER_UNLOCKED;

static void onI2CRequest() {
  BlobPacket snap;
  portENTER_CRITICAL_ISR(&g_pktMux);
  snap = g_pkt;
  portEXIT_CRITICAL_ISR(&g_pktMux);

  Wire.write(reinterpret_cast<const uint8_t*>(&snap), sizeof(snap));
}

static volatile uint8_t g_last_cmd = 0;
static void onI2CReceive(int nBytes)
{
  if (nBytes <= 0) return;
  g_last_cmd = (uint8_t)Wire.read();
  while (Wire.available()) (void)Wire.read();
}

static void i2c_slave_init()
{
  Wire.begin(I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);
}

static void i2c_publish_blob(const BlobResult &b) {
  portENTER_CRITICAL(&g_pktMux);

  uint32_t next = g_pkt.seq + 1;
  if (next == 0) next = 1;

  g_pkt.seq = next;
  if (b.found) {
    g_pkt.valid = 1;
    g_pkt.cx = b.cx;
    g_pkt.cy = b.cy;
    g_pkt.bw = (float)(b.xmax - b.xmin + 1);
    g_pkt.bh = (float)(b.ymax - b.ymin + 1);
//    Serial.print( g_pkt.cx, 3);
//    Serial.print(",");
//    Serial.println( g_pkt.cy, 3);
  } else {
    g_pkt.valid = 0;
    g_pkt.cx = 0.0f;
    g_pkt.cy = 0.0f;
    g_pkt.bw = 0.0f;
    g_pkt.bh = 0.0f;
  }

  portEXIT_CRITICAL(&g_pktMux);
}

// ===================== Dual-core frame pipeline =====================
static QueueHandle_t g_frame_q = nullptr;
static constexpr int FRAME_Q_LEN = 2;          // keep small to avoid latency buildup
static constexpr bool DROP_OLD_FRAMES = true;  // always prefer newest frames

static void capture_task(void *param)
{
  (void)param;

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // If queue full, drop policy
    if (uxQueueSpacesAvailable(g_frame_q) == 0) {
      if (DROP_OLD_FRAMES) {
        camera_fb_t *old = nullptr;
        if (xQueueReceive(g_frame_q, &old, 0) == pdTRUE && old) {
          esp_camera_fb_return(old);
        }
      } else {
        esp_camera_fb_return(fb);
        continue;
      }
    }

    if (xQueueSend(g_frame_q, &fb, 0) != pdTRUE) {
      esp_camera_fb_return(fb);
    }
  }
}

static void process_task(void *param)
{
  (void)param;

  while (true) {
    camera_fb_t *fb = nullptr;
    if (xQueueReceive(g_frame_q, &fb, portMAX_DELAY) != pdTRUE || !fb) continue;

    BlobResult b = find_red_blob_rgb565(fb, STRIDE);
    i2c_publish_blob(b);

    // Optional: serial debug at low rate
    //if (b.found) Serial.printf("Blob cx=%.1f cy=%.1f bw=%.1f bh=%.1f cnt=%u\n", b.cx, b.cy, (float)(b.xmax-b.xmin+1), (float)(b.ymax-b.ymin+1), (unsigned)b.count);
    if (b.found) Serial.printf("%.1f, %.1f\n", b.cx, b.cy );

    esp_camera_fb_return(fb);
  }
}

// ===================== Camera init =====================
static bool init_camera_rgb565()
{
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.frame_size  = FRAMESIZE_QVGA;       // 320x240
    config.fb_count    = 2;                    // important for pipelining
    config.grab_mode   = CAMERA_GRAB_LATEST;
    g_min_count        = MIN_COUNT_QVGA_STR2;
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.frame_size  = FRAMESIZE_QQVGA;      // 160x120
    config.fb_count    = 1;
    config.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
    g_min_count        = MIN_COUNT_QQVGA_STR2;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    // If lighting is stable, consider disabling these for more stable thresholds:
    // s->set_whitebal(s, 0);
    // s->set_awb_gain(s, 0);
    // s->set_gain_ctrl(s, 0);
    // s->set_exposure_ctrl(s, 0);
  }

  return true;
}

// ===================== Arduino entry =====================
void setup()
{
  Serial.begin(115200);
  delay(200);

  Serial.printf("PSRAM: %s\n", psramFound() ? "YES" : "NO");
  Serial.printf("I2C slave: 0x%02X SDA=%d SCL=%d @%lu\n",
                I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, (unsigned long)I2C_FREQ_HZ);

  i2c_slave_init();

  if (!init_camera_rgb565()) {
    Serial.println("Camera init failed. Halting.");
    while (true) delay(1000);
  }

  g_frame_q = xQueueCreate(FRAME_Q_LEN, sizeof(camera_fb_t*));
  if (!g_frame_q) {
    Serial.println("Failed to create frame queue. Halting.");
    while (true) delay(1000);
  }

  // Core assignment:
  // - Core 0 often has more system/WiFi work; if you see issues, swap cores.
  xTaskCreatePinnedToCore(capture_task, "capture", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(process_task, "process", 8192, nullptr, 1, nullptr, 1);

  Serial.printf("Frame size: %s, min_count=%lu, stride=%d\n",
                psramFound() ? "QVGA" : "QQVGA",
                (unsigned long)g_min_count, STRIDE);
}

void loop()
{
  // All work done in tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
