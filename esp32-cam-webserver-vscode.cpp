/*  Author Antony Nolan 
 *  This sketch is a rework of the reworking of the 'official' ESP32 Camera example sketch from Expressif:
 *  https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer and
 *  https://github.com/easytarget/esp32-cam-webserver
 *  
 *  Additional FUnctionality
 *  - Brownout Condition Removal
 *  - TTGO - All board versions supported
 *  - Additional Relays added
 *  - I2C LCD support 
 *  - IR Sensor with Auto Image capture Support
 *  
 * 
 *  For personal use mostly. _It is modified to allow control of Illumination LED Lamps's (present on some modules),
 *  greater feedback via a status LED, and the HTML contents are present in plain text
 *  for easy modification. 
 *  
 *  A camera name can now be configured, and wifi details can be stored in an optional 
 *  header file to allow easier updated of the repo.
 *  
 *  The web UI has had minor changes to add the lamp control when present, I have made the 
 *  'Start Stream' controls more accessible, and add feedback of the camera name/firmware.
 *  
 * note: Make sure that you have either selected ESP32 AI Thinker,
 * or another board which has PSRAM enabled to use high resolution camera modes. 
 * There are lots of vendors that produce ESP32 with a camera with different characteristics. 
 * 
 * Supported PSRAM ESP Modules
 * 4MB
 * - ESP32-CAM AI-Thinker 
 * - M5CAMERA
 * - M5STACK
 * 8MB
 * - TTGO PIR Motion, audio and i2C LCD version 17.2
 * 
 */

/* 
 *  FOR NETWORK AND HARDWARE SETTINGS COPY OR RENAME 'myconfig.sample.h' to 'myconfig.h' AND EDIT THAT.
 * By default this sketch will assume an AI-THINKER ESP-CAM and create 
 * an accesspoint called "ESP32-CAM-CONNECT" (password: "InsecurePassword")
 *
 */

/*  
 ######### myconfig.h contains the #definition entries exaluated in the if statements below
*/

// Custom Configuration Entries
#include "myconfig.h"

// Network Configuration
#include <WiFi.h>
#include <ArduinoWebsockets.h>

// Template and Board Camera Access
#include "esp_camera.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"

// Platformio
#include "Arduino.h"

//Disable Brownout Errors
#include "soc/soc.h"          // Disable brownour problems
#include "soc/rtc_cntl_reg.h" // Disable brownour problems

// Memory Internal and SD
#include "driver/rtc_io.h"
#include <EEPROM.h> // read and write from flash memory
#include "FS.h"     // SD Card ESP32
#include "SD_MMC.h" // SD Card ESP32

// ############################ Do not edit the Primary config  #######################################

// ###############Copy myconfig.sample.h to myconfig.h and edit that file

#warning "Using Default Settings: Copy myconfig.sample.h to myconfig.h and edit that to set your personal defaults"

// These are the defaults.. dont edit these.
//  SSID, Password and Mode
const char *ssid = "ESP32-CAM-CONNECT";
const char *password = "InsecurePassword";
#define WIFI_AP_ENABLE
// Default Board and Camera:
#define CAMERA_MODEL_AI_THINKER

// Pin Mappings
#include "camera_pins.h"

// Declare external function from app_httpd.cpp
extern void startCameraServer(int hPort, int sPort);

// A Name for the Camera. (set in myconfig.h)
#if defined(CAM_NAME)
char myName[] = CAM_NAME;
#else
char myName[] = "ESP32 camera server";
#endif

// Ports for http and stream (override in myconfig.h)
#if defined(HTTP_PORT)
int httpPort = HTTP_PORT;
#else
int httpPort = 80;
#endif

#if defined(STREAM_PORT)
int streamPort = STREAM_PORT;
#else
int streamPort = 81;
#endif

// The stream URL
char streamURL[64] = {"Undefined"}; // Stream URL to pass to the app.

// This will be displayed to identify the firmware
char myVer[] PROGMEM = __DATE__ " @ " __TIME__;

// initial rotation
// can be set in myconfig.h
#if !defined(CAM_ROTATION)
#define CAM_ROTATION 0
#endif
int myRotation = CAM_ROTATION;

// Illumination LAMP/LED
#if defined(LAMP_DISABLE)
int lampVal = -1; // lamp is disabled in config
#elif defined(LAMP_PIN)
#if defined(LAMP_DEFAULT)
int lampVal = constrain(LAMP_DEFAULT, 0, 100); // initial lamp value, range 0-100
#else
int lampVal = 0; //default to off
#endif
#else
int lampVal = -1; // no lamp pin assigned
#endif

int lampChannel = 7;         // a free PWM channel (some channels used by camera)
const int pwmfreq = 50000;   // 50K pwm frequency
const int pwmresolution = 9; // duty cycle bit range
const int pwmMax = pow(2, pwmresolution) - 1;

#if defined(FACE_DETECTION)
int8_t detection_enabled = 1;
#if defined(FACE_RECOGNITION)
int8_t recognition_enabled = 1;
#else
int8_t recognition_enabled = 0;
#endif
#else
int8_t detection_enabled = 0;
int8_t recognition_enabled = 0;
#endif

// Notification LED
void flashLED(int flashtime)
{
#ifdef LED_PIN                    // If we have it; flash it.
  digitalWrite(LED_PIN, LED_ON);  // On at full power.
  delay(flashtime);               // delay
  digitalWrite(LED_PIN, LED_OFF); // turn Off
#else
  return; // No notifcation LED, do nothing, no delay
#endif
}
#define ALARM_PIN 2 // pin GPIO2 
#define PIR_PIN 12 // pin GPIO12
unsigned long currentMillis = 0;
unsigned long openedMillis = 0;
long interval = 5000;           // open lock for ... milliseconds




// Alarm Relay LED
void ALARMRELAY(int alarmOn)
{
#ifdef ALARM_PIN                     // Initiate Lock PowerOn Test
  digitalWrite(ALARM_PIN, ALARM_ON); // On at full power.
  delay(RELAYtime);                  // delay
  digitalWrite(LED_PIN, LED_OFF);    // turn Off
#else
  return; // No alarm LED, do nothing, no delay
#endif
}

// Lamp Control
void setLamp(int newVal)
{
  if (newVal != -1)
  {
    // Apply a logarithmic function to the scale.
    int brightness = round((pow(2, (1 + (newVal * 0.02))) - 2) / 6 * pwmMax);
    ledcWrite(lampChannel, brightness);
    Serial.print("Lamp: ");
    Serial.print(newVal);
    Serial.print("%, pwm = ");
    Serial.println(brightness);
  }
}
// ################## VOID SETUP ###################

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("====");
  Serial.print("esp32-cam-webserver: ");
  Serial.println(myName);
  Serial.print("Code Built: ");
  Serial.println(myVer);

  // Create camera config structure; and populate with hardware and other defaults
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
  //init with highest supported specs to pre-allocate large buffers
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init ##########################################################
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // Dump camera module, warn for unsupported modules.
  switch (s->id.PID)
  {
  case OV9650_PID:
    Serial.println("WARNING: OV9650 camera module is not properly supported, will fallback to OV2640 operation");
    break;
  case OV7725_PID:
    Serial.println("WARNING: OV7725 camera module is not properly supported, will fallback to OV2640 operation");
    break;
  case OV2640_PID:
    Serial.println("OV2640 camera module detected");
    break;
  case OV3660_PID:
    Serial.println("OV3660 camera module detected");
    break;
  // case OV5640_PID: Serial.println("WARNING: OV5640 camera module is not properly supported, will fallback to OV2640 operation"); break;
  default:
    Serial.println("WARNING: Camera module is unknown and not properly supported, will fallback to OV2640 operation");
  }

  // OV3660 initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       //flip it back
    s->set_brightness(s, 1);  //up the blightness just a bit
    s->set_saturation(s, -2); //lower the saturation
  }

// M5 Stack Wide has special needs
#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

// Config can override mirror and flip
#if defined(H_MIRROR)
  s->set_hmirror(s, H_MIRROR);
#endif
#if defined(V_FLIP)
  s->set_vflip(s, V_FLIP);
#endif

// set initial frame rate
#if defined(DEFAULT_RESOLUTION)
  s->set_framesize(s, DEFAULT_RESOLUTION);
#else
  s->set_framesize(s, FRAMESIZE_QVGA);
#endif

  /*
   * Add any other defaults you want to apply at startup here:
   * uncomment the line and set the value as desired (see the comments)
   */
  //s->set_brightness(s, 0);     // -2 to 2
  //s->set_contrast(s, 0);       // -2 to 2
  //s->set_saturation(s, 0);     // -2 to 2
  //s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  //s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  //s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  //s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  //s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  //s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  //s->set_ae_level(s, 0);       // -2 to 2
  //s->set_aec_value(s, 300);    // 0 to 1200
  //s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  //s->set_agc_gain(s, 0);       // 0 to 30
  //s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  //s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  //s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  //s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  //s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  //s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  //s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  //s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  //s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

  // We now have our config defined; setup the hardware.

  // Initialise and set the lamp
  if (lampVal != -1)
  {
    ledcSetup(lampChannel, pwmfreq, pwmresolution); // configure LED PWM channel
    setLamp(lampVal);                               // set default value
    ledcAttachPin(LAMP_PIN, lampChannel);           // attach the GPIO pin to the channel
  }
  else
  {
    Serial.println("No lamp, or lamp disabled in config");
  }

  // Feedback that we are now attempting to connect
  Serial.println();
  Serial.println("Wifi Initialisation");
  flashLED(400);
  delay(100);

#if defined(WIFI_AP_ENABLE)
#if defined(AP_ADDRESS)
  IPAddress local_IP(AP_ADDRESS);
  IPAddress gateway(AP_ADDRESS);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
#endif
#if defined(AP_CHAN)
  WiFi.softAP(ssid, password, AP_CHAN);
  Serial.println("Setting up Fixed Channel AccessPoint");
  Serial.print("SSID     : ");
  Serial.println(ssid);
  Serial.print("Password : ");
  Serial.println(password);
  Serial.print("Channel  : ");
  Serial.println(AP_CHAN);
#else
  WiFi.softAP(ssid, password);
  Serial.println("Setting up AccessPoint");
  Serial.print("SSID     : ");
  Serial.println(ssid);
  Serial.print("Password : ");
  Serial.println(password);
#endif
#else
  Serial.print("Connecting to Wifi Network: ");
  Serial.println(ssid);
#if defined(ST_IP)
#if !defined(ST_GATEWAY) || !defined(ST_NETMASK)
#error "You must supply both Gateway and NetMask when specifying a static IP address"
#endif
  IPAddress staticIP(ST_IP);
  IPAddress gateway(ST_GATEWAY);
  IPAddress subnet(ST_NETMASK);
#if !defined(ST_DNS1)
  WiFi.config(staticIP, gateway, subnet);
#else
  IPAddress dns1(ST_DNS1);
#if !defined(ST_DNS2)
  WiFi.config(staticIP, gateway, subnet, dns1);
#else
  IPAddress dns2(ST_DNS2);
  WiFi.config(staticIP, gateway, subnet, dns1, dns2);
#endif
#endif
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(250); // Wait for Wifi to connect. If this fails wifi the code basically hangs here.
                // - It would be good to do something else here as a future enhancement.
                //   (eg: go to a captive AP config portal to configure the wifi)
  }

  // feedback that we are connected
  Serial.println("WiFi connected");
  flashLED(200);
  delay(100);
  flashLED(200);
  delay(100);
  flashLED(200);
#endif

  // Start the Stream server, and the handler processes for the Web UI.
  startCameraServer(httpPort, streamPort);

  IPAddress ip;
  char httpURL[64] = {"Unknown"};

#if defined(WIFI_AP_ENABLE)
  ip = WiFi.softAPIP();
#else
  ip = WiFi.localIP();
#endif

  // Construct the App URL
  if (httpPort != 80)
  {
    sprintf(httpURL, "http://%d.%d.%d.%d:%d/", ip[0], ip[1], ip[2], ip[3], httpPort);
  }
  else
  {
    sprintf(httpURL, "http://%d.%d.%d.%d/", ip[0], ip[1], ip[2], ip[3]);
  }

  // Construct the Stream URL
  sprintf(streamURL, "http://%d.%d.%d.%d:%d/", ip[0], ip[1], ip[2], ip[3], streamPort);

  Serial.printf("\nCamera Ready!\nUse '%s' to connect\n", httpURL);
  Serial.printf("Raw stream URL is '%s'\n", streamURL);
  Serial.printf("Stream viewer available at '%sview'", streamURL);

  face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  // Read current face data from on-board flash
  read_face_id_from_flash(&id_list);

  void rzoCheckForFace()
  {
    currentMillis = millis();
    if (run_face_recognition())
    { // face recognition function has returned true
      Serial.println("Face recognised");
      digitalWrite(relayPin, HIGH); //close (energise) relay
      openedMillis = millis();      //time relay closed
    }
    if (currentMillis - interval > openedMillis)
    {                              // current time - face recognised time > 5 secs
      digitalWrite(relayPin, LOW); //open relay
    }

    bool run_face_recognition()
    {
      bool faceRecognised = false; // default
      int64_t start_time = esp_timer_get_time();
      fb = esp_camera_fb_get();
      if (!fb)
      {
        Serial.println("Camera capture failed");
        return false;
      }

      int64_t fb_get_time = esp_timer_get_time();
      Serial.printf("Get one frame in %u ms.\n", (fb_get_time - start_time) / 1000); // this line can be commented out

      image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
      uint32_t res = fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
      if (!res)
      {
        Serial.println("to rgb888 failed");
        dl_matrix3du_free(image_matrix);
      }

      esp_camera_fb_return(fb);

      box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);

      if (net_boxes)
      {
        if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK)
        {

          int matched_id = recognize_face(&id_list, aligned_face);
          if (matched_id >= 0)
          {
            Serial.printf("Match Face ID: %u\n", matched_id);
            faceRecognised = true; // function will now return true
          }
          else
          {
            Serial.println("No Match Found");
            matched_id = -1;
          }
        }
        else
        {
          Serial.println("Face Not Aligned");
        }

        free(net_boxes->box);
        free(net_boxes->landmark);
        free(net_boxes);
      }

      dl_matrix3du_free(image_matrix);
      return faceRecognised;
    }
  }
  // ################## VOID LOOP ###################

  void loop()
  {
    // Just loop forever
    // All function prototypes loaded into and open loop upon execution
    // The stream and URI handler processes initiated by the startCameraServer() call at the
    // end of setup() will handle the camera and UI processing from now on.
    delay(10000);
  }
