/*ECE 590: ENGINEERING PROJECT II
 * MULTI-PURPOSE QUADCOPTER
 * : Gas Quality/Leakage Monitoring
 * EC/09/14
 * KITHINJI MURIUNGI
 * 2018/19
 * Supervisor: Mr. S.B.Kifalu
*/

//MQ2 GAS SENSOR
#include <ESP8266WiFi.h> //
#include <FirebaseArduino.h>

// Set these to run example.
/*#define FIREBASE_HOST "droneiotgasmonitor.firebaseio.com"
#define FIREBASE_AUTH "J45Fu0kzUjCUwaKZ8Dg83GFIKF6NBpD4es7NofM5"*/


#define FIREBASE_HOST "home-automation-7fc48.firebaseio.com" 
#define FIREBASE_AUTH "KKDcaIEcTWOHGvHHdtlemDkf1Q37alqqpUcwbSt2" 

#define WIFI_SSID "Panthers" //Your Wi-Fi SSID
#define WIFI_PASSWORD "qwerty90" //Your Wi-Fi Password

const int mq2GasSensor = A0; // Gas Sensor

void setup() {
  Serial.begin(9600);

  pinMode(mq2GasSensor, INPUT); 

  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}

void loop() {
  float newGasReading = analogRead(mq2GasSensor);
    Firebase.setFloat("air_condition", newGasReading);
    
  delay(200);
}/*END OF THE CODE
