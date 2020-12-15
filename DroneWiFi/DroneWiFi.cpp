/* ***************************************************************** */
/* File name:        DroneWiFi.cpp                                   */
/* File description: WiFi socket connection handling implementation  */
/* Author name:      Giacomo Dollevedo, Gustavo Fernandes            */
/* Creation date:    18nov2020                                       */
/* Revision date:    14dec2020                                       */
/* ***************************************************************** */

#include "DroneWiFi.h"


/* ************************************************************************************ */
/* Method's name:          initWiFi                                                     */ 
/* Description:            WiFi initialization. Connects to a network and server.       */
/*                         Authenticates.                                               */
/*                                                                                      */
/* Entry parameters:       char* ssid -> WiFi network ssid                              */
/*                         char* pass -> WiFi network password                          */
/*                         char* hostIP -> Server connection IP                         */
/*                         int port    -> Server Connection port                        */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void DroneWiFi::initWiFi(char* ssid, char* pass, char* hostIP, int port)
{

  connectWifi(ssid, pass);

  unsigned char flag = 0;
  
  while(flag == 0){
    flag = connectServer(hostIP, port);
  }
  
  if(debugging_enabled){
    Serial.println("Connected to Server!");
    Serial.println("Sending Authentication...");
  }
  receiveData();
  delay(100);
  sendData( "OK\0");
  if(debugging_enabled)
    Serial.println("\nAuthentication successful!");

}

/* ************************************************************************************ */
/* Method's name:          sendData                                                     */ 
/* Description:            WiFi comm. method to send data. Will mark the first 5 bytes  */
/*                         with the message length for the receiver end.                */
/*                                                                                      */
/* Entry parameters:       WiFiClient _serverCon -> Socket object that represents the   */
/*                                            connection                                */
/*                         String message -> Message that will be sent through socket   */
/*                         char* len_c -> Array to enable the first 5 bytes marker      */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void DroneWiFi::sendData(String message)       // aceita um registro como parâmetro
{
  int   len = int(message.length());
  unsigned int i = 0;

  sprintf(_len_marker, "%05d", len); // 5 digits

  String envio = _len_marker + message;
  _serverCon.print(envio);
  return;

}


/* ************************************************************************************ */
/* Method's name:          receiveData                                                  */ 
/* Description:            WiFi comm. method to receive data. Will read the first 5     */
/*                         incoming bytes and use it to read the incoming stream        */
/*                                                                                      */
/* Entry parameters:       WiFiClient _serverCon -> Socket object that represents the   */
/*                                            connection                                */
/*                                                                                      */
/* Return parameters:      String -> Message read from the connection                   */
/* ************************************************************************************ */
String DroneWiFi::receiveData()
{
  String read_size = "";
  int i = 0;
  int n;
  byte message[249];

  while (!_serverCon.available() || _serverCon.available() <= 4)
    delay(1);
  if (_serverCon.available() > 4) {
    while (i < 5) {
      read_size = read_size + char(_serverCon.read());
      i++;
    }

    n = read_size.toInt();
    i = 0;
    read_size = "";
  }

  while (!_serverCon.available() || _serverCon.available() <= (n - 1))
    delay(1);

  if (_serverCon.available() > 0) {
    while (i < n) {
      read_size = read_size + char(_serverCon.read());
      i++;
    }
    //Serial.println(read_size);
    if(debugging_enabled)
      Serial.println(read_size);
    return read_size;
  }
  
}

/* ************************************************************************************ */
/* Method's name:          setParams                                                    */ 
/* Description:            Set internal droneParams variable                            */
/*                                                                                      */
/* Entry parameters:       droneParams prm -> struct to be set                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void DroneWiFi::setParams(droneParams prm)
{

  _params = prm;
  return;


}


/* ************************************************************************************ */
/* Method's name:          getParams                                                    */ 
/* Description:            Return the internal drone parameters struct                  */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      droneParams -> internal parameters struct                    */
/* ************************************************************************************ */
droneParams DroneWiFi::getParams()
{
 return _params;
}

unsigned char DroneWiFi::connectWifi(char* ssid, char* pass)
{
  _wifiCon.addAP(ssid, pass); // Network name and password

  if(debugging_enabled){
    Serial.println();
    Serial.println();
    Serial.print("Waiting for WiFi... ");
  }
  while (_wifiCon.run() != WL_CONNECTED) {
    if(debugging_enabled)
      Serial.print(".");
    delay(500);
  }

  if(debugging_enabled)
    Serial.println("\nConnected to WiFi!");

  return 1;

}


/* ************************************************************************************ */
/* Method's name:          connectServer                                                */ 
/* Description:            Attempts server connection through WiFi network              */
/*                                                                                      */
/* Entry parameters:       char* hostIP -> Server connection IP                         */
/*                         int port    -> Server Connection port                        */
/*                                                                                      */
/* Return parameters:      unsigned char -> 0 == failed to connect / 1 == success       */
/* ************************************************************************************ */
unsigned char DroneWiFi::connectServer(char* hostIP, int port)
{
  if (!_serverCon.connect(hostIP, port)) {
    if(debugging_enabled){
      Serial.println("Connection failed.");
      Serial.println("Waiting 5 seconds before retrying...");
    }
    delay(5000);
    return 0;
  }

  else{
    return 1;
  }
}

/* ************************************************************************************ */
/* Method's name:          processComm                                                  */ 
/* Description:            State machine to process server communication commands       */
/*                                                                                      */
/* Entry parameters:       String msg -> server command to be handled                   */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void DroneWiFi::processComm(String msg)
{
  int len = (int)msg.length();
  int i = 0;
  char ch;
  
  if(joystick_enabled){
    switch (msg[0]){

    case 's':
      joystick_enabled = 0;
      if(debugging_enabled)
        Serial.println("Joystick Disabled.");
      break;
    
    case '5':
      _joystickSetpoints.roll   = 0;
      _joystickSetpoints.pitch  = 0;  
      break;

    case '8':
      _joystickSetpoints.pitch  -= VEL_INC;
      if(_joystickSetpoints.pitch <= -10)
        _joystickSetpoints.pitch = -10;
     
      break;
    
    case '2':
      _joystickSetpoints.pitch  += VEL_INC;
      if(_joystickSetpoints.pitch >= 10)
        _joystickSetpoints.pitch = 10;
      break;

    case '6':
      _joystickSetpoints.roll   += VEL_INC;
      if(_joystickSetpoints.roll >= 10)
        _joystickSetpoints.roll = 10;

      break;

    case '4':
      _joystickSetpoints.roll   -= VEL_INC;
      if(_joystickSetpoints.roll <= -10)
        _joystickSetpoints.roll = -10;
      break;

    case '+':
      _joystickSetpoints.throttle   += VEL_INC;
      if(_joystickSetpoints.throttle >= 50)
        _joystickSetpoints.throttle = 50;
      break;

    case '-':
      _joystickSetpoints.throttle   -= VEL_INC;
      if(_joystickSetpoints.throttle <= 0)
        _joystickSetpoints.throttle = 0;
      break;

    case '#':
      break;

    default:
      if(debugging_enabled)
        Serial.println("Unindentified Command.");
      break;
    }
    
  }

  else{
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

          if(msg[i-1] == 'S')
            if(len == 22){
              _params.M1 = (msg[3]-48)*1000 + (msg[4]-48)*100 + (msg[5]-48)*10 + (msg[6]-48);
              _params.M2 = (msg[8]-48)*1000 + (msg[9]-48)*100 + (msg[10]-48)*10 + (msg[11]-48);
              _params.M3 = (msg[13]-48)*1000 + (msg[14]-48)*100 + (msg[15]-48)*10 + (msg[16]-48);
              _params.M4 = (msg[18]-48)*1000 + (msg[19]-48)*100 + (msg[20]-48)*10 + (msg[21]-48);
            }
          i = len;
          break;
        
        case 'G':

          if(debugging_enabled)
            Serial.println("Main Loop Started!");
          sendData("Ready to fly!\0");
          i++;
          break;

        case 'V':
          if(len == 7){
            _params.setPoint = (msg[3]-48)*10 + (msg[4]-48);
            _params.time = (msg[6]-48);
          }

          i = len;
          break;

        case 'R':

          _params.M1 = 1200;
          _params.M2 = 1200;
          _params.M3 = 1200;
          _params.M4 = 1200;

          i = len;
          break;

        case 'J':
          joystick_enabled = 1;
          i++;
          if(debugging_enabled)
            Serial.println("Joystick Enabled.");
          break;

        default:
          if(debugging_enabled)
            Serial.println("Unindentified Command.");
          i = len;
      }
    }
  }

  return;
}

/* ************************************************************************************ */
/* Method's name:          getVel                                                       */ 
/* Description:            Return the internal joystick setpoints struct                */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      rotVel -> internal joystick setpoints struct                 */
/* ************************************************************************************ */
rotVel DroneWiFi::getVel(){
  return _joystickSetpoints;
}

/* ************************************************************************************ */
/* Method's name:          enable_debug                                                 */ 
/* Description:            Enables Serial Comm Printing                                 */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void DroneWiFi::enable_debug(){
  debugging_enabled = 1;
}

/* ************************************************************************************ */
/* Method's name:          disable_debug                                                */ 
/* Description:            Disables Serial Comm Printing                                */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void DroneWiFi::disable_debug(){
  debugging_enabled = 0;
}