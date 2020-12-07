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

typedef struct droneParams
 {
  int M1, M2, M3, M4;
  float H, Battery;
  int setPoint;
  int time;

 };


class DroneWiFi
{
public:
 void initWiFi(char* ssid, char* pass, char* hostIP, int port);
 void sendData(String message, char* len_c);
 String receiveData();
 void setParams(droneParams prm);
 void processComm(String msg);
 droneParams getParams();


private:

  WiFiMulti _wifiCon;
  WiFiClient _serverCon;

  droneParams _params;

  unsigned char connectWifi(char* ssid, char* pass);
  unsigned char connectServer(char* hostIP, int port);
  char* _len_marker = (char*)calloc(5, sizeof(char));
  

  
};
 
#endif
