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


typedef struct droneParams
 {
  int M1, M2, M3, M4;
  float H, Battery;

 };


class DroneWiFi
{
public:
 void initWiFi(char* ssid, char* pass, char* hostIP, int port);
 void sendData(WiFiClient conn, String message, char* len_c);
 String receiveData(WiFiClient conn);
 void setParams(droneParams prm);
 droneParams getParams();


private:

  WiFiMulti wifiCon;
  WiFiClient serverCon;

  droneParams params;

  char*       len_marker = (char*)calloc(5, sizeof(char));
  int         sent_counter    =   0;
  char*       message = (char*)calloc(1024, sizeof(char));


  unsigned char connectWifi(char* ssid, char* pass);
  unsigned char connectServer(char* hostIP, int port);
  void processComm(String msg);

  
};
 
#endif
