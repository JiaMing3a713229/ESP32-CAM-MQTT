#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <Arduino.h>
#include <SPIFFS.h>
#include <FS.h>
#include <SD_MMC.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"

//ESP32-CAM Pin Mapping
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
// ESP32-Cam configure of image Capture 
#define FPS 3
#define ESP_CAM_FPS (1 / FPS) * 1000

// ESP32 MQTT 
WiFiClient ESP32MQTT;
PubSubClient client(ESP32MQTT);
#define WIFI_ENV_ROOM8    1
#define WIFI_ENV_ICLAB    2
#define WIFI_ENV_JIAPHONE 3

#define WIFI_ENV WIFI_ENV_ROOM8  // 在這裡設置當前的 WiFi 環境

#if WIFI_ENV == WIFI_ENV_ROOM8
    #define WIFI_SSID "501"
    #define WIFI_PASS "66580501"
#elif WIFI_ENV == WIFI_ENV_ICLAB
    #define WIFI_SSID "ICLAB_SSID"
    #define WIFI_PASS "ICLAB_PASS"
#elif WIFI_ENV == WIFI_ENV_JIAPHONE
    #define WIFI_SSID "JIAPHONE_SSID"
    #define WIFI_PASS "JIAPHONE_PASS"
#else
    #error "Invalid WIFI_ENV setting"
#endif

const char* ssid = WIFI_SSID;
const char* pass = WIFI_PASS;

//Jiaphone
const char* mqtt_server = "192.168.139.106";
const int mqttPort = 1883; 
const char* clientID = "ESPCam-1";
long lastReconnectAttempt;
//設置MQTT TOPIC 針對1號機資料上傳MQTT
const char* channel1_image = "machine/camera/jpeg_image";
String msgStr = "";

//init camera
void camera_Init(){
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
    config.frame_size = FRAMESIZE_VGA;
    config.pixel_format = PIXFORMAT_JPEG; // for streaming
    //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    
    // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
    // for larger pre-allocated frame buffer.
    if(config.pixel_format == PIXFORMAT_JPEG){
        if(psramFound()){
            //config.frame_size = FRAMESIZE_UXGA;
            config.frame_size = FRAMESIZE_SVGA;
            config.jpeg_quality = 13;
            config.fb_count = 2;
        } 
        else {
        // Limit the frame size when PSRAM is not available
            //config.frame_size = FRAMESIZE_HVGA;
            config.frame_size = FRAMESIZE_SVGA;
            config.jpeg_quality = 13;
            config.fb_count = 1;
            config.fb_location = CAMERA_FB_IN_DRAM;
        }
    } else {
        // Best option for face detection/recognition
        config.frame_size = FRAMESIZE_240X240;
    }
    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
    Serial.println("Camera Init Successed!");
}
esp_err_t camera_captureTOMQTT(){
    camera_fb_t *fb = esp_camera_fb_get();
    if(!fb){
        Serial.println("Camera capture failed!");
        ESP.restart();
        return ESP_FAIL;
    }
    int ps = MQTT_MAX_PACKET_SIZE;
    // // start to publish the picture
    client.beginPublish(channel1_image, fb->len, false);
    for (int i = 0; i < fb->len; i += ps) {
      int s = (fb->len - i < s) ? (fb->len- i) : ps;
      client.write((uint8_t *)(fb->buf) + i, s);
    }
    // uint8_t *fbBuf = fb->buf;
    // size_t fbLen = fb->len;
    // client.beginPublish(channel1_image, fbLen, 0);
    // for(int i=0; i<fbLen; i+=ps){
    //   if((i+1024) < fbLen){
    //     client.write((uint8_t *)fbBuf + i, ps);
    //   }
    //   else{
    //     size_t remainder = fbLen%1024;
    //     client.write((uint8_t *)(fb->buf)+i, remainder);
    //   }
    // }
    client.endPublish();
    esp_camera_fb_return(fb);
    return ESP_OK; 
}

//-------------------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int lenght){
  String payload_buff;
  for(int i=0; i<lenght; i++){
    payload_buff = payload_buff + String((char)payload[i]);
  }
  Serial.printf("mqtt receive: %s\n",payload_buff);
}
void reconnect() {
  while(!client.connected()){
    if (client.connect(clientID)) {
      client.subscribe(channel1_image); // Subscribe to channel.
    }
  }
}

//mqtt task
void taskMQTT(void * pvParams){
  client.setServer(mqtt_server, mqttPort);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  camera_Init();
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  while(1){
    if(!client.connected()){
        reconnect();
    }
    else{
      //發送資料
      client.loop();
      camera_captureTOMQTT();
      vTaskDelay(ESP_CAM_FPS / portTICK_PERIOD_MS);    // 延遲ESP_CAM_FPSms
    }
  }
}
//wifi連線task
void ConnectWIFITask(){
  int i = 0;
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS && i<50);
    i++;
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("Wifi Connected, Start mqtt task");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());
    Serial.println("Start MQTT Task!");
    xTaskCreate(taskMQTT, "taskMQTT", 1024 * 12, NULL, 1, NULL);
  }
  else{
    Serial.println("WiFi 連線已逾時!");
    ESP.restart();
  }
  
}
void setup() {
  Serial.begin(115200);
  ConnectWIFITask();
}

void loop() {
  // put your main code here, to run repeatedly:
}