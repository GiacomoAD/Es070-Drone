
#include <WiFi.h>
#include <string.h>
String readString;

#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>
#include <WiFi.h>
void setup() {


    server.begin();
    Serial.begin(9600);
    connectToWifi();
    m1.attach(22, 1000, 2000); 
}
 
void loop() {

}
