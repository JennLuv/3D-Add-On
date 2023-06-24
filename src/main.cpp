#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Wire.h>

#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "Base64.h"

//https://github.com/zenmanenergy/ESP8266-Arduino-Examples/
String urlencode(String str)
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
}

String Photo2Base64() {
    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();  
    if(!fb) {
      Serial.println("Camera capture failed");
      return "";
    }
  
    String imageFile = "data:image/jpeg;base64,";
    char *input = (char *)fb->buf;
    char output[base64_enc_len(3)];
    for (int i=0;i<fb->len;i++) {
      base64_encode(output, (input++), 3);
      if (i%3==0) imageFile += urlencode(String(output));
    }

    esp_camera_fb_return(fb);
    
    return imageFile;
}


//CAMERA_MODEL_AI_THINKER
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

//define the sensor pins
#define hallSensorPin 15
#define switchPin 14

// Provide the token generation process info.
#include "addons/TokenHelper.h"

// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "Jennibear"
#define WIFI_PASSWORD "itsasecretlol"

// Insert Firebase project API Key
#define API_KEY "AIzaSyBVQcLbOlwnloA21Z61rvbBC4AZtxulAaE"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "tereterepashi@gmail.com"
#define USER_PASSWORD "tereterepashi123"

// Insert RTDB URLefine the RTDB URL
#define DATABASE_URL "https://esp-cam-1b487-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid; 

// Variables to save database paths
/*String databasePath;
String tempPath;
String humPath;
String presPath;
float temperature;
float humidity;
float pressure;*/
String databasePath;
String hallSensorPath;
String switchPath;
String photoPath;
int hallSensorPinInput;
int switchPinInput;
bool erordetect;
String photoInput;
String alertInput;
String alertPath;
String rotPath;
String statePath;
String lastStatePath;
String cntOnPath;
String cntOffPath;
String counterOnPath;
String counterOffPath;

// Timer variables (send new readings every three minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 1000; //this i think 1 sec?

// Initialize WiFi
void initWiFi() {
 WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 Serial.print("Connecting to WiFi ..");
 while (WiFi.status() != WL_CONNECTED) {
 Serial.print('.');
 delay(100);
 }
 Serial.println(WiFi.localIP());
 Serial.println();
}

// Write float values to the database
void sendInt(String path, int value){
 if (Firebase.RTDB.setInt(&fbdo, path.c_str(), value)){
 Serial.print("Writing value: ");
 Serial.print (value);
 Serial.print(" on the following path: ");
 Serial.println(path);
 Serial.println("PASSED");
 Serial.println("PATH: " + fbdo.dataPath());
 Serial.println("TYPE: " + fbdo.dataType());
 }
 else {
 Serial.println("FAILED");
 Serial.println("REASON: " + fbdo.errorReason());
 }
}

//this is the sendstring new code
void sendstring(String path, String str){
 if (Firebase.RTDB.setString(&fbdo, path.c_str(), str)){
 Serial.print("Writing value: ");
 Serial.print (str);
 Serial.print(" on the following path: ");
 Serial.println(path);
 Serial.println("PASSED");
 Serial.println("PATH: " + fbdo.dataPath());
 Serial.println("TYPE: " + fbdo.dataType());
 }
 else {
 Serial.println("FAILED");
 Serial.println("REASON: " + fbdo.errorReason());
 }
}

void sendAlert(String path, String str){
 if (Firebase.RTDB.setString(&fbdo, path.c_str(), str)){
 Serial.print("Writing value: ");
 Serial.print (str);
 Serial.print(" on the following path: ");
 Serial.println(path);
 Serial.println("PASSED");
 Serial.println("PATH: " + fbdo.dataPath());
 Serial.println("TYPE: " + fbdo.dataType());
 }
 else {
 Serial.println("FAILED");
 Serial.println("REASON: " + fbdo.errorReason());
 }
}

void sendData(){
 // Get latest sensor readings
 hallSensorPinInput = digitalRead(hallSensorPin);
 //hallSensorPinInput = 1;
 photoInput = Photo2Base64(); // must be edited

  // Send readings to database:
 sendInt(hallSensorPath, hallSensorPinInput);
 sendstring(photoPath, photoInput);
}

void initCamera(){
 
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

 if (psramFound()) {
   config.frame_size = FRAMESIZE_UXGA;
   config.jpeg_quality = 10;
   config.fb_count = 2;
 } else {
   config.frame_size = FRAMESIZE_SVGA;
   config.jpeg_quality = 12;
   config.fb_count = 1;
 }
 // Camera init
 esp_err_t err = esp_camera_init(&config);
 if (err != ESP_OK) {
  Serial.printf("Camera init failed with error 0x%x", err);
  ESP.restart();
 }
}

int hall = 0;
int counterOn, counterOff, state, lastState;
int tolerance = 30;
int cntOn = 1000;
int cntOff = 1000;
int rotation = 0;
int fullRotation = 0;

void counter(){
  delay(500); //wait for firebase to initialize
  hall = digitalRead(hallSensorPin);
  //hall = 1;
  
  if (hall == 1){
    if (counterOff != 0){
      cntOff = counterOff;
    }
    counterOn = counterOn + 1;
    /*Serial.println("hall: ");
    Serial.println(hall);
    Serial.println("counterOn: ");
    Serial.println(counterOn);
    Serial.println("cntOff: ");
    Serial.println(cntOff);
    Serial.println("cntOn: ");
    Serial.println(cntOn);
    Serial.println("counterOff: ");
    Serial.println(counterOff);*/
    sendInt(counterOnPath, counterOn);
    sendInt(counterOffPath, counterOff);
    sendInt(cntOnPath, cntOn);
    sendInt(cntOffPath, cntOff);
    
    lastState = state;
    sendInt(statePath, state);
    sendInt(lastStatePath, lastState);
    /*Serial.println("lastState:");
    Serial.println(lastState);*/
    state = 1;
    /*Serial.println("State: ");
    Serial.println(state);*/
    hall = digitalRead(hallSensorPin);
    counterOff = 0;
    alertInput = "nothing's wrong";
    sendAlert(alertPath, alertInput);
    
    if (counterOn > cntOn + tolerance){
      alertInput = "Alert in 1 state, too slow";
      sendAlert(alertPath, alertInput);
    }
    
    
  } else if (hall == 0) {
        if (counterOn != 0){
      cntOn = counterOn;
      }
      counterOff = counterOff + 1;
      /*Serial.println("hall: ");
      Serial.println(hall);
      Serial.println("counterOff: ");
      Serial.println(counterOff);
      Serial.println("cntOn: ");
      Serial.println(cntOn);
      Serial.println("cntOff: ");
      Serial.println(cntOff);
      Serial.println("counterOn: ");
      Serial.println(counterOn);*/
      sendInt(counterOnPath, counterOn);
      sendInt(counterOffPath, counterOff);
      sendInt(cntOnPath, cntOn);
      sendInt(cntOffPath, cntOff);

      lastState = state;
      sendInt(statePath, state);
      sendInt(lastStatePath, lastState);
      /*Serial.println("lastState:");
      Serial.println(lastState);*/
      state = 0;
      /*Serial.println("State: ");
      Serial.println(state);*/
      hall = digitalRead(hallSensorPin);
      counterOn = 0;
      alertInput = "nothing's wrong";
      sendAlert(alertPath, alertInput);

      if (counterOff > cntOff + tolerance){
        alertInput = "Alert in 0 state, too slow";
        sendAlert(alertPath, alertInput);
      } 
    }
  if ((lastState == 0 and state == 1) or (lastState == 1 and state == 0)){
    if (rotation>= 2 and counterOff < cntOff -tolerance){
        alertInput = "Alert in 0 state, too fast";
        sendAlert(alertPath, alertInput);
    }else if (rotation >= 2 and counterOn < cntOn -tolerance){
      alertInput = "Alert in 1 state, too fast";
      sendAlert(alertPath, alertInput);
    }
    rotation += 1;
    fullRotation = rotation/4;
    sendInt(rotPath, fullRotation);
  }
}


void setup(){
 Serial.begin(115200);
 initWiFi();
 initCamera();

 sensor_t * s = esp_camera_sensor_get();
 s->set_framesize(s, FRAMESIZE_QQVGA);  // VGA|CIF|QVGA|HQVGA|QQVGA   ( UXGA? SXGA? XGA? SVGA? )
 
 //disable brownout detector
 WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG,0);
 // Assign the api key (required)
 config.api_key = API_KEY;
 // Assign the user sign in credentials
 auth.user.email = USER_EMAIL;
 auth.user.password = USER_PASSWORD;
 // Assign the RTDB URL (required)
 config.database_url = DATABASE_URL;
 Firebase.reconnectWiFi(true);
 fbdo.setResponseSize(4096);
 // Assign the callback function for the long running token generation task */
 config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
 // Assign the maximum retry of token generation
 config.max_token_generation_retry = 5;
 // Initialize the library with the Firebase authen and config
 Firebase.begin(&config, &auth);
 // Getting the user UID might take a few seconds
 Serial.println("Getting User UID");
 while ((auth.token.uid) == "") {
 Serial.print('.');
 delay(1000);
 }

 // sensor pinModes
 pinMode(hallSensorPin,INPUT);
 pinMode(switchPin, INPUT);
 
 // Print user UID
 uid = auth.token.uid.c_str();
 Serial.print("User UID: ");
 Serial.println(uid);
 
 // Update database path
 databasePath = "/UsersData/" + uid;
 
 // Update database path for sensor readings
 hallSensorPath = databasePath + "/hallSensor"; // --> UsersData/<user_uid>/hallSensor
 photoPath = databasePath + "/esp32-cam";
 alertPath = databasePath + "/alert";
 rotPath = databasePath + "/rot";
 statePath = databasePath + "/state";
 counterOnPath = databasePath + "/counterOnPath";
 counterOffPath = databasePath + "/counterOffPath";
 cntOffPath = databasePath + "/cntOffPath";
 cntOnPath = databasePath + "/cntOnPath";
 statePath = databasePath + "/statePath";
 lastStatePath = databasePath + "/lastStatePath";

 alertInput = "nothing's wrong";
 sendAlert(alertPath, alertInput);
}

void loop(){
 // Send new readings to database
 if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)){
 sendDataPrevMillis = millis();
  sendData();
  counter();
 }
}