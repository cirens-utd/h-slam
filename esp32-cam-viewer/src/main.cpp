// Streams JPEG frames over USB serial.
// Each frame: "FRAME" + 4-byte little-endian length + JPEG bytes.
// On boot, tries known ESP32-S3 camera board pinouts until one finds a sensor.
#include <Arduino.h>
#include "esp_camera.h"

struct CamPins {
  const char *name;
  int pwdn, reset, xclk, sda, scl;
  int d7, d6, d5, d4, d3, d2, d1, d0;  // Y9..Y2
  int vsync, href, pclk;
};

static const CamPins BOARDS[] = {
  {"ESP32-S3-EYE / Freenove S3 CAM", -1, -1, 15, 4, 5, 16, 17, 18, 12, 10, 8, 9, 11, 6, 7, 13},
  {"Seeed XIAO ESP32S3 Sense", -1, -1, 10, 40, 39, 48, 11, 12, 14, 16, 18, 17, 15, 38, 47, 13},
  {"M5Stack CamS3", -1, 21, 11, 17, 41, 13, 4, 10, 5, 7, 16, 15, 6, 42, 18, 12},
  {"LilyGo T-Camera S3", -1, 39, 38, 5, 4, 9, 10, 11, 13, 21, 48, 47, 14, 8, 18, 12},
  {"DFRobot FireBeetle 2 S3", -1, -1, 45, 1, 2, 48, 46, 8, 7, 4, 41, 40, 39, 6, 42, 5},
};

static const CamPins *active = nullptr;
static String report;

static esp_err_t tryInit(const CamPins &p) {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = p.d0; c.pin_d1 = p.d1; c.pin_d2 = p.d2; c.pin_d3 = p.d3;
  c.pin_d4 = p.d4; c.pin_d5 = p.d5; c.pin_d6 = p.d6; c.pin_d7 = p.d7;
  c.pin_xclk = p.xclk;
  c.pin_pclk = p.pclk;
  c.pin_vsync = p.vsync;
  c.pin_href = p.href;
  c.pin_sccb_sda = p.sda;
  c.pin_sccb_scl = p.scl;
  c.pin_pwdn = p.pwdn;
  c.pin_reset = p.reset;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;  // 640x480
  c.jpeg_quality = 12;           // 0-63, lower = better quality
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  return esp_camera_init(&c);
}

void setup() {
  Serial.begin(921600);  // baud is ignored on native USB, runs at full USB speed
  report = "INFO: PSRAM " + String(ESP.getPsramSize() / 1024) + " KB\n";

  for (const CamPins &p : BOARDS) {
    esp_err_t err = tryInit(p);
    report += "INFO: " + String(p.name) + " -> " + esp_err_to_name(err) + "\n";
    if (err == ESP_OK) {
      active = &p;
      break;
    }
    esp_camera_deinit();
    delay(50);
  }
}

void loop() {
  if (!active) {
    Serial.print(report);
    Serial.println("ERROR: camera init failed on all known pinouts (check ribbon cable)");
    delay(2000);
    return;
  }

  static uint32_t lastInfo = 0;
  if (millis() - lastInfo > 5000) {  // periodically say which board matched
    lastInfo = millis();
    String msg = "INFO: using " + String(active->name) + "\n";
    Serial.print(msg);
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  uint32_t len = fb->len;
  Serial.write("FRAME", 5);
  Serial.write((uint8_t *)&len, 4);
  Serial.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}
