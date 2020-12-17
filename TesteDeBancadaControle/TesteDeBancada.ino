#include "DroneWiFi.h"
#include "IMU.h"
#include "DroneTimer.h"
#include "FlightControl.h"
#include "ThrottleControl.h"

#define FREQUENCY 900
#define TRIGGER_200MS FREQUENCY/5
#define TRIGGER_100MS FREQUENCY/10
#define TRIGGER_CONTROL   FREQUENCY/225

int         sent_counter  =   0;
char*       message       = (char*)calloc(1024, sizeof(char));
String      command       = "";

volatile unsigned char base_counter = 0;
volatile unsigned char imu_trigger = 0;
volatile unsigned char wifi_trigger = 0;
volatile unsigned char control_trigger = 0;

int* actualVel = (int*) calloc(3, sizeof(int));
int vel1, vel2, vel3, vel4;



DroneTimer timer;
DroneWiFi wifi;
IMU imu;
FlightControl pitchVelPid(2.1f, 0.5f, 3.0f, 'r');
ThrottleControl quadcopter;


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

  if(base_counter >= TRIGGER_100MS){
    base_counter = 0;
    wifi_trigger = 1;
  }

  if(base_counter >= TRIGGER_CONTROL){
      control_trigger = 1;

  }
}




void setup() {
  Serial.begin(115200);
  delay(10);

  timer.initTimer(FREQUENCY, time_count);
  imu.initMPU();

  wifi.disable_debug();
  wifi.initWiFi(SSID_GUS, PASS_GUS, IP_GIA, PORT);

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

    pitchVelPid.setSetPoint(0);
    quadcopter.initializeMotors(13,12,14,27);

    imu.disableDebug();

}

void loop() {

  actualVel = quadcopter.getThrottle();
  vel1 = actualVel[0];
  vel2 = actualVel[1];
  vel3 = actualVel[2];
  vel4 = actualVel[3];

  Serial.println(vel1);
  Serial.println(vel2);
  Serial.println(vel3);
  Serial.println(vel4);
  

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
      pitchVelPid.setSetPoint(setpoints.pitch);
      
    }
  
    sprintf(message, "Angles: Roll:%.2f\tPitch:%.2f\tSetpoint: Roll:%d\tPitch:%d\tThrottle:%d\0", rotations.Roll, rotations.Pitch, setpoints.roll, setpoints.pitch, setpoints.throttle);
    wifi.sendData(message);
    sent_counter++;
  
    if(sent_counter == 5){
      sent_counter = 0;
    }

    if(control_trigger){ 
        control_trigger = 0;
        //Executa a rotina de controle para o eixo de roll 
        pitchVelPid.pidControl(imu.getRotations());
        quadcopter.SingleAxisVelControl(pitchVelPid);
        

    }
    
  }



}
