#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
float gps_latitude = 0;
float gps_longitude = 0;
float gps_speed = 0;
float gps_altitude = 0;
static const int RXPin = 4, TXPin = 0;
static const uint32_t GPSBaud = 9600;
//String lat_str, lng_str;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

#include "MPU9250.h"
#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#include <FirebaseESP8266.h>
#endif

// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "Dassir"
#define WIFI_PASSWORD "12345678"

// For the following credentials, see examples/Authentications/SignInAsUser/EmailPassword/EmailPassword.ino

/* 2. Define the API Key */
#define API_KEY "AIzaSyBWVYTAVD-nz1jjfjbZv-D_FNao_wKHgSA"

/* 3. Define the RTDB URL */
#define DATABASE_URL "https://track-6aeb6-default-rtdb.asia-southeast1.firebasedatabase.app/" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app

/* 4. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "systemewarn@gmail.com"
#define USER_PASSWORD "interns@ewarn"

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;


unsigned long sendDataPrevMillis = 0;
unsigned long getDataPrevMillis = 0;
bool signupOK = false;

MPU9250 mpu;
int tracking = 1;
int accident_happened = 0;
int accident_happened0 = 0;


int accident_detected = 0;

float YAW_new;
float PITCH_new;
float ROLL_new;

float YAW_previous;
float PITCH_previous;
float ROLL_previous;

float YAW_difference;
float PITCH_difference;
float ROLL_difference;

float ACCX;
float ACCY;
float ACCZ;

float max_difference = 10.0;

float ALA;
float ALA_THRESH = 2;
float PITCH_THRESH = 45;
float ROLL_THRESH = 45;
unsigned long startTime;
unsigned long endTime;
const int communication_pin1 = 23;
const int communication_pin2 = 15;

void Taskmpu( void *pvParameters );
void gps_firebase( void *pvParameters );

// the setup function runs once when you press reset or power the board
void setup() {


  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  // Or use legacy authenticate method
  // config.database_url = DATABASE_URL;
  // config.signer.tokens.legacy_token = "<database secret>";

  // To connect without auth in Test Mode, see Authentications/TestMode/TestMode.ino

  //////////////////////////////////////////////////////////////////////////////////////////////
  // Please make sure the device free Heap is not lower than 80 k for ESP32 and 10 k for ESP8266,
  // otherwise the SSL connection will fail.
  //////////////////////////////////////////////////////////////////////////////////////////////

  Firebase.begin(&config, &auth);

  // Comment or pass false value when WiFi reconnection will control by your code or third party library
  Firebase.reconnectWiFi(true);

  Firebase.setDoubleDigits(5);
  // Now set up two tasks to run independently.

  pinMode(communication_pin1, OUTPUT);
  digitalWrite(communication_pin1, LOW);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  // calibrate anytime you want to

  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();

  delay(5000);
  mpu.calibrateMag();


  mpu.verbose(false);

  pinMode(communication_pin2, INPUT);

  ss.begin(GPSBaud);
  Serial.println(TinyGPSPlus::libraryVersion());

  xTaskCreatePinnedToCore(
    Taskmpu
    ,  "Taskmpu"   // A name just for humans
    ,  20000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  //
    ,  NULL
    ,  0);

  xTaskCreatePinnedToCore(
    gps_firebase
    ,  "gps_firebase"
    ,  85000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL
    ,  1);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void Taskmpu(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  Serial.print("Setup: mpu9250Executing on core ");
  Serial.println(xPortGetCoreID());
  int count = 0;
  

  // initialize digital LED_BUILTIN on pin 13 as an output.


  for (;;) // A Task shall never return or exit.
  {
    if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {

        YAW_new = mpu.getYaw();
        PITCH_new = mpu.getPitch();
        ROLL_new = mpu.getRoll();
        YAW_difference = abs(YAW_new - YAW_previous);
        PITCH_difference = abs(PITCH_new - PITCH_previous);
        ROLL_difference = abs(ROLL_new - ROLL_previous);
        ACCX = mpu.getAccX();
        ACCY = mpu.getAccY();
        ACCZ = mpu.getAccZ();


        ALA = sqrt(sq(ACCX) + sq(ACCY) + sq(ACCZ));
        //Serial.println(ALA);
        //Serial.println(ROTATION_FILTERED);
        if (ALA > ALA_THRESH || PITCH_difference > max_difference || ROLL_difference > max_difference || abs(PITCH_new) > PITCH_THRESH || abs(ROLL_new) > ROLL_THRESH || ALA < 0.1 || YAW_difference > 45 ) {
          if (count != 0) {
            digitalWrite(communication_pin1, HIGH);
            accident_happened0 = 1;
            startTime =  millis();
            Serial.println("ACCIDENT");
          }
          else {
            count = 1;
          }
        }

        else {

          if (accident_happened0 == 1) {
            endTime = millis();

            if (endTime - startTime > 20000) {
              digitalWrite(communication_pin1, LOW);
              accident_happened0 = 0;
              delay(10);
            }

          }

        }
        YAW_previous = YAW_new;
        PITCH_previous = PITCH_new;
        ROLL_previous = ROLL_new;

        prev_ms = millis();
      }
    }// one tick delay (15ms) in between reads for stability
  }
}

void gps_firebase(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  Serial.print("Setup: gps and firebase Executing on core ");
  Serial.println(xPortGetCoreID());


  

  for (;;)
  { smartdelay_gps(1000);
    gps_latitude = gps.location.lat();
    gps_longitude = gps.location.lng();
    gps_altitude = gps.altitude.meters();
    gps_speed = gps.speed.kmph();



    if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring"));


    if (Firebase.ready() && (millis() - getDataPrevMillis > 15000 || getDataPrevMillis == 0)) {
      getDataPrevMillis = millis();
      Firebase.getInt(fbdo, F("monitoring/auto_monitoring"), &tracking);

      if (fbdo.errorReason() == "connection lost") {
        WiFi.disconnect();
        Serial.println("reconnecting wifi will start sfter 5 seconds");
        delay(5000);
        WiFi.reconnect();
        Serial.println();
        Serial.print("Reonnected with IP: ");
        Serial.println(WiFi.localIP());
        Serial.println();

      }



    }
    if (Firebase.ready() && tracking == 1 && digitalRead(communication_pin2) == HIGH && accident_detected == 0 ) {
      sendDataPrevMillis = millis();
      accident_happened = accident_happened + 1;
      accident_detected = 1;


      Serial.printf("Set int... %s\n", Firebase.setInt(fbdo, F("Accident/number of accidents"), accident_happened) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set int... %s\n", Firebase.setInt(fbdo, F("Accident/new accident status"), accident_detected) ? "ok" : fbdo.errorReason().c_str());


      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/latitude"), gps_latitude ) ? "ok" : fbdo.errorReason().c_str());


      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/longitude"), gps_longitude ) ? "ok" : fbdo.errorReason().c_str());


      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/altitude"), gps_altitude ) ? "ok" : fbdo.errorReason().c_str());


      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/speed(kmph)"), gps_speed) ? "ok" : fbdo.errorReason().c_str());

      delay(500);


    }
    if (Firebase.ready() && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0) && tracking == 0 && accident_detected == 0) {
      sendDataPrevMillis = millis();


      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/latitude"), gps_latitude ) ? "ok" : fbdo.errorReason().c_str());


      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/longitude"), gps_longitude ) ? "ok" : fbdo.errorReason().c_str());


      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/altitude"), gps_altitude ) ? "ok" : fbdo.errorReason().c_str());


      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/speed(kmph)"), gps_speed) ? "ok" : fbdo.errorReason().c_str());

      delay(500);

    }
    if (Firebase.ready() && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0) && tracking == 1 && accident_detected == 0 ) {
      sendDataPrevMillis = millis();
      Serial.printf("Set int... %s\n", Firebase.setInt(fbdo, F("Accident/number of accidents"), accident_happened) ? "ok" : fbdo.errorReason().c_str());

      Serial.printf("Set int... %s\n", Firebase.setInt(fbdo, F("Accident/new accident status"), accident_detected) ? "ok" : fbdo.errorReason().c_str());



      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/latitude"), gps_latitude ) ? "ok" : fbdo.errorReason().c_str());


      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/longitude"), gps_longitude ) ? "ok" : fbdo.errorReason().c_str());


      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/altitude"), gps_altitude ) ? "ok" : fbdo.errorReason().c_str());


      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("location/speed(kmph)"), gps_speed) ? "ok" : fbdo.errorReason().c_str());


      delay(500);

    }
    if (digitalRead(communication_pin2) == LOW) {
      accident_detected = 0;

      delay(500);
    }


  }
}



static void smartdelay_gps(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
