#include <DroneWiFi.h>
#include <IMU.h>
#include <DroneTimer.h>

#define FREQUENCY 50
#define TRIGGER_200MS FREQUENCY/5


int         sent_counter  =   0;
char*       message       = (char*)calloc(1024, sizeof(char));
String      command       = "";

volatile unsigned char base_counter = 0;
volatile unsigned char imu_trigger = 0;
volatile unsigned char wifi_trigger = 0;



DroneTimer timer;
DroneWiFi wifi;
IMU imu;

mpu rawData;
processedAngles rotations;
int calibrar = 0;
float calx  = 0;
float caly  = 0;
float calz  = 0;
float calAx = 0;
float calAy = 0;

rotVel setpoints;

void IRAM_ATTR time_count(){

  base_counter++;

  if(imu_trigger == 0)
    imu_trigger = 1;

  if(base_counter >= TRIGGER_200MS){
    base_counter = 0;
    wifi_trigger = 1;
  }
  

}




void setup() {
  Serial.begin(115200);
  delay(10);

  timer.initTimer(FREQUENCY, time_count);
  imu.initMPU();
  wifi.initWiFi(SSID_GIA, PASS_GIA, IP_GIA, PORT);

  setpoints = wifi.getVel();
  delay(10);
  timer.enableTimer();

  while(calibrar < 2000){
    if(imu_trigger){
      imu_trigger = 0;
      imu.readRawMPU();
      rawData = imu.getRawData();

      calx  += rawData.GyX;
      caly  += rawData.GyY;
      calz  += rawData.GyZ;
      calAx += rawData.AcX;
      calAy += rawData.AcY;
      
      calibrar++;
    }
  }

  if(calibrar < 2500){    
    imu.CalibrateGyro(calx/calibrar, caly/calibrar, calz/calibrar);
    imu.CalibrateAcl(calAx/calibrar, calAy/calibrar);
    calibrar = 3000;
  }


}

void loop() {

  if(imu_trigger){
    imu_trigger = 0;
    imu.update();
  }


  if(wifi_trigger){
    wifi_trigger = 0;

    rotations = imu.getRotations();
    
    if(sent_counter == 0){
      command = wifi.receiveData();
      wifi.processComm((String)command);
      setpoints = wifi.getVel();
    }
  
    sprintf(message, "Angles: Roll:%.2f\tPitch:%.2f\tSetpoint: Roll:%d\tPitch:%d\0", rotations.Roll, rotations.Pitch, setpoints.roll, setpoints.pitch);
    wifi.sendData(message);
    sent_counter++;
  
    if(sent_counter == 5){
      sent_counter = 0;
    }
    
  }

  
  
  delay(100);

}
