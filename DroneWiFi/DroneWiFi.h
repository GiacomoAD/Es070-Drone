/*
 DroneWiFi.h 
 
*/
 
#ifndef DroneWiFi_h
#define DroneWiFi_h

#include <WiFi.h>
#include <WiFiMulti.h>

//#include <Arduino.h>

#define IP_GIA "192.168.43.182" 
#define PORT 25565
#define SSID_GIA "Gio"
#define PASS_GIA "nematodeo"
#define SSID_GUS "Guslifer-Wifi"
#define PASS_GUS "12345678"

#define VEL_INC 5

typedef struct droneParams
 {
  int M1, M2, M3, M4;
  float H, Battery;
  int setPoint;
  int time;

 };

typedef struct rotVel
 {
  int roll = 0;
  int pitch = 0;
  int yaw = 0;

 };

class DroneWiFi
{
public:
 void initWiFi(char* ssid, char* pass, char* hostIP, int port);
 void sendData(String message);
 String receiveData();
 void setParams(droneParams prm);
 void processComm(String msg);
 droneParams getParams();
 rotVel getVel();
 void enable_debug();
 void disable_debug();

private:

  WiFiMulti _wifiCon;
  WiFiClient _serverCon;

  droneParams _params;
  rotVel  _joystickSetpoints;

  unsigned char connectWifi(char* ssid, char* pass);
  unsigned char connectServer(char* hostIP, int port);
  char* _len_marker = (char*)calloc(5, sizeof(char));
  
  unsigned char joystick_enabled = 0;
  unsigned char debugging_enabled = 1;
  
};
 
#endif
