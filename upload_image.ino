#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <ESP32Servo.h>

Servo servoFeed;
Servo servoClean;

ESP32PWM pwm;

int feed = 0, clean = 0;

int sudutServoFeed[2] = {
  0,
  45
};

//Sesuaikan sudut servo dengan alat
int sudutServoClean[2] = {
  0,
  90
};

// Replace with your network credentials
const char* ssid     = "vivo 1918";
const char* password = "12345678";

// database pembacaan sensor
short int httpResponseCode = 0;

// Pembacaan Sensor
unsigned int updateTime = 5000, lastUpdate = 0;
unsigned int updateTimeServo = 2000, lastUpdateServoFeed = 0, lastUpdateServoClean = 0;

// CAMERA_MODEL_AI_THINKER
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

void wifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(",");
  }
}

void cameraInit() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
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
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    Serial.println("found psram");
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.println("camera init failed");
    ESP.restart();
    WiFi.disconnect();
  }
}

String espServer() {
  const char* serverName = "http://pet.feeding.mboisjatimbarat.com/esp-server.php";
  String apiKeyServer = "tPmAT5Ab3j7F9";

  HTTPClient http;
  WiFiClient serverClient;
  http.begin(serverClient, serverName);

  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  // Prepare your HTTP POST request data
  String httpRequestData = "api_key=" + apiKeyServer;
  Serial.print("httpRequestData: "); Serial.println(httpRequestData);

  // Send HTTP POST request
  httpResponseCode = http.POST(httpRequestData);
  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println(payload);

    // return dari fungsi
    // output yang diinginkan = camReq-clean-feed
    return payload;
  }
  else Serial.println("Server tidak merespon");
  // Free resources
  serverClient.stop();
  http.end();
  delay(1);
}

void sendPhoto() {
  WiFiClient imageClient;
  // database to upload image
  String serverImage = "pet.feeding.mboisjatimbarat.com";
  String serverPath = "/upload.php";
  const int serverPort = 80;
  String getAll, getBody;

  //  mengambil foto
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    delay(1000);
    ESP.restart();
  }

  Serial.println(imageClient.connect(serverImage.c_str(), serverPort));
  if (imageClient.connect(serverImage.c_str(), serverPort)) {
    Serial.println("Connection successful!");
    String head = "--PetFeeding\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--PetFeeding--\r\n";

    //body dari content yang dikirimkan
    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;

    //  ****pengiriman content****
    imageClient.println("POST " + serverPath + " HTTP/1.1");
    imageClient.println("Host: " + serverImage);
    imageClient.println("Content-Length: " + String(totalLen));
    imageClient.println("Content-Type: multipart/form-data; boundary=PetFeeding");
    imageClient.println();

    //print head
    imageClient.print(head);

    //print image data
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        imageClient.write(fbBuf, 1024);
        fbBuf += 1024;
      } else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        imageClient.write(fbBuf, remainder);
      }
    }

    //print tail
    imageClient.print(tail);
    esp_camera_fb_return(fb);

    //read body response from server
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;

    while ((millis() - startTimer) <  timoutTimer) {
      Serial.print(".");
      delay(100);
      while (imageClient.available()) {
        Serial.println("getting feedback");
        char c = imageClient.read();
        if (c == '\n') {
          if (getAll.length() == 0) state = true;
          getAll = "";
        }
        else if (c != '\r') getAll += String(c);

        if (state) getBody += String(c);
        startTimer = millis();
      }
      if (getBody.length() > 0) break;
    }
    
    imageClient.stop();
  } else Serial.println("connection failed");
}

void setup() {
   // put your setup code here, to run once:
  Serial.begin(115200);

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoFeed.setPeriodHertz(50);      // Standard 50hz servo
  servoClean.setPeriodHertz(50);      // Standard 50hz servo
  int minUs = 500;
  int maxUs = 2400;

  servoFeed.attach(14, minUs, maxUs);
  servoClean.attach(15, minUs, maxUs);
  servoFeed.write(sudutServoFeed[0]);
  servoClean.write(sudutServoClean[0]);
  wifiInit();
  cameraInit();
}

void loop() {
  if (millis() - lastUpdate > updateTime) {
    servoFeed.write(sudutServoFeed[0]);
    servoClean.write(sudutServoClean[0]);
    
    // PARSING PAYLOAD        
    String payload = espServer();
    int ind = payload.indexOf('-');
    String camReq = payload.substring(0, ind);
    int ind2 = payload.indexOf('-', ind + 1);
    feed = payload.substring(ind + 1, ind2).toInt();
    clean = payload.substring(ind2 + 1).toInt();
    
    Serial.print("payload");Serial.println(payload);
    Serial.print("cam");Serial.println(camReq);
    Serial.print("feed");Serial.println(feed);
    Serial.print("clean");Serial.println(clean);
    
    // CEK ISI PAYLOAD
    if (camReq == "1") {
      sendPhoto();
      camReq == "0";
    }
    if (feed == 1) {
      Serial.print("feed"); Serial.println(sudutServoFeed[feed]);
      servoFeed.write(sudutServoFeed[feed]);
      lastUpdateServoFeed = millis();
    }
    if (clean == 1) {
      Serial.print("clean"); Serial.println(sudutServoClean[clean]);
      servoClean.write(sudutServoClean[clean]);
      lastUpdateServoClean = millis();
    }
    
    lastUpdate = millis();
    Serial.println("delay program 5 detik");
  }

  if (clean == 1) {
    if (millis() - lastUpdateServoClean > updateTimeServo) {
      Serial.println("back to 0");
      clean = 0;
      servoClean.write(sudutServoClean[clean]);
    }
  }
  if (feed == 1) {
    if (millis() - lastUpdateServoFeed > updateTimeServo) {
      Serial.println("back to 0");
      feed = 0;
      servoFeed.write(sudutServoFeed[feed]);
    }
  }
}
