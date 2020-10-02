#include "esp_http_client.h"
#include <WiFi.h>
#include "esp_camera.h"
#include "Arduino.h"
#include "fd_forward.h"


const char* ssid = "NSA";
const char* password = "Orange";
char* sonoff_ip = "http://192.168.1.103";
long switch_time_out = 5000;

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

long last_detected_millis = 0;
bool switch_status = 0;

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.7;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 4;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.4;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

static esp_err_t update_sonoff_status()
{
  char* full_sonoff_address = (char*)malloc(40);
  sprintf(full_sonoff_address, "%s/cm?cmnd=Power%%20TOGGLE", sonoff_ip);

  esp_http_client_config_t config = {
    .url = full_sonoff_address,
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);
  esp_err_t err = esp_http_client_perform(client);

  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Status = %d, content_length = %d",
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));
  }
  esp_http_client_cleanup(client);
}

void loop()
{
  camera_fb_t * fb = NULL;
  dl_matrix3du_t *image_matrix = NULL;
  
  while (true) {

    fb = esp_camera_fb_get();
    image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

    fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);

    box_array_t *net_boxes = NULL;
    net_boxes = face_detect(image_matrix, &mtmn_config);

    if (net_boxes) {
      last_detected_millis = millis();
      if (switch_status == 0) { // switch is off
        Serial.println("Face Seen");
        update_sonoff_status();
        switch_status = 1;
      }
    }

    esp_camera_fb_return(fb);
    fb = NULL;
    dl_matrix3du_free(image_matrix);

    if (millis() - last_detected_millis > switch_time_out && switch_status == 1) { // No face detected and switch is on
      Serial.println("No Faces for switch_time_out milliseconds");
      switch_status = 0;
      update_sonoff_status();
    }

  }

}
