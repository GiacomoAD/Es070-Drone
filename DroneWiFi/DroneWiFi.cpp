#include "DroneWiFi.h"

void DroneWiFi::initWiFi(char* ssid, char* pass, char* hostIP, int port)      //aceita um registro e um valor como parâmetro
{

  connectWifi(ssid, pass);

  unsigned char flag = 0;
  
  while(flag == 0){
    flag = connectServer(hostIP, port);
  }
  
  Serial.println("Connected to Server!");
  Serial.println("Sending Authentication...");
  receiveData(serverCon);
  delay(100);
  sendData(serverCon, "OK\0", len_marker);
  Serial.println("\nAuthentication successful!");

}

/* ************************************************************************************ */
/* Method's name:          sendData                                                     */ 
/* Description:            WiFi comm. method to send data. Will mark the first 5 bytes  */
/*                         with the message length for the receiver end.                */
/*                                                                                      */
/* Entry parameters:       WiFiClient conn -> Socket object that represents the         */
/*                                            connection                                */
/*                         String message -> Message that will be sent through socket   */
/*                         char* len_c -> Array to enable the first 5 bytes marker      */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void DroneWiFi::sendData(WiFiClient conn, String message, char* len_c)       // aceita um registro como parâmetro
{
  int   len = int(message.length());
  unsigned int i = 0;

  sprintf(len_c, "%05d", len); // 5 digits

  String envio = len_c + message;
  conn.print(envio);
  return;

}


/* ************************************************************************************ */
/* Method's name:          receiveData                                                  */ 
/* Description:            WiFi comm. method to receive data. Will read the first 5     */
/*                         incoming bytes and use it to read the incoming stream        */
/*                                                                                      */
/* Entry parameters:       WiFiClient conn -> Socket object that represents the         */
/*                                            connection                                */
/*                                                                                      */
/* Return parameters:      String -> Message read from the connection                   */
/* ************************************************************************************ */
String DroneWiFi::receiveData(WiFiClient conn)
{
  String read_size = "";
  int i = 0;
  int n;
  byte message[249];

  while (!conn.available() || conn.available() <= 4)
    delay(1);
  if (conn.available() > 4) {
    while (i < 5) {
      read_size = read_size + char(conn.read());
      i++;
    }

    n = read_size.toInt();
    i = 0;
    read_size = "";
  }

  while (!conn.available() || conn.available() <= (n - 1))
    delay(1);

  if (conn.available() > 0) {
    while (i < n) {
      read_size = read_size + char(conn.read());
      i++;
    }
    //Serial.println(read_size);
    Serial.println(read_size);
    return read_size;
  }
  
}

void DroneWiFi::setParams(droneParams prm)
{

  params = prm;
  return;


}

droneParams DroneWiFi::getParams()
{
 return params;
}

unsigned char DroneWiFi::connectWifi(char* ssid, char* pass)
{
  wifiCon.addAP(ssid, pass); // Network name and password

  Serial.println();
  Serial.println();
  Serial.print("Waiting for WiFi... ");
  
  while (wifiCon.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nConnected to WiFi!");

  return 1;

}

unsigned char DroneWiFi::connectServer(char* hostIP, int port)
{
  if (!serverCon.connect(hostIP, port)) {
    Serial.println("Connection failed.");
    Serial.println("Waiting 5 seconds before retrying...");
    delay(5000);
    return 0;
  }

  else{
    return 1;
  }
}

void DroneWiFi::processComm(String msg)
{
  int len = (int)msg.length();
  int i = 0;
  char ch;
  
  while(i < len){
    ch = msg[i];
    switch(ch){
      case '#':
        i++;
        break;
      case 'K':
        i++;
        break;
      case 'S':
        i++;
        break;
      //#ST1000;1000;1000;1000
      case 'T':
        if(len == 22){
          params.M1 = (msg[3]-48)*1000 + (msg[4]-48)*100 + (msg[5]-48)*10 + (msg[6]-48);
          params.M2 = (msg[8]-48)*1000 + (msg[9]-48)*100 + (msg[10]-48)*10 + (msg[11]-48);
          params.M3 = (msg[13]-48)*1000 + (msg[14]-48)*100 + (msg[15]-48)*10 + (msg[16]-48);
          params.M4 = (msg[18]-48)*1000 + (msg[19]-48)*100 + (msg[20]-48)*10 + (msg[21]-48);
        }
        i = len;
        break;
      
      case 'G':
        Serial.println("Main Loop Started!");
        sendData(serverCon, "Ready to fly!\0", len_marker);
        i++;
        break;

      default:
        Serial.println("Unindentified Command.");
        i = len;
    }
  }

  return;

}
