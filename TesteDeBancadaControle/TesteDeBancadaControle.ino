/*****************************************************************************\
 * Descrição: Main para testes em bancada de calibração do controlador PID   *
 * do Drone                                                                  *
 * Autores: Gustavo L. Fernandes e Giácomo A. Dollevedo                      *                    
 * Ultima Atualização: 30/12/2020                                            *
******************************************************************************/

#include "DroneWiFi.h"
#include "IMU.h"
#include "DroneTimer.h"
#include "FlightControl.h"
#include "ThrottleControl.h"

#define FREQUENCY 4000
#define TRIGGER_200MS FREQUENCY/5
#define TRIGGER_100MS FREQUENCY/40
#define TRIGGER_500MS FREQUENCY/2000
#define TRIGGER_CONTROL   FREQUENCY/16

int         sent_counter  =   0;
char*       message       = (char*)calloc(1024, sizeof(char));
String      command       = "";

volatile unsigned char base_counter = 0;
volatile unsigned char imu_trigger = 0;
volatile unsigned char wifi_trigger = 0;
volatile unsigned char control_trigger = 0;

int* actualVel = (int*) calloc(3, sizeof(int));
int vel1, vel2, vel3, vel4;
pidGains desiredGains;



DroneTimer timer;
DroneWiFi wifi;
IMU imu;
FlightControl pitchVelPid(0.5f, 0.8f, 0.9f , 'p');
ThrottleControl quadcopter;


mpu rawData;
processedAngles rotations;
int calibrar = 0;
float calx  = 0;
float caly  = 0;
float calz  = 0;
float calAx = 0;
float calAy = 0;

droneParams prm ;
int counterControl = 0;
int counterWifi = 0;

rotVel setpoints;
float addTrottleTemp;

void IRAM_ATTR time_count(){

  base_counter++;
  counterControl++;
  counterWifi++;

  if(imu_trigger == 0)
    imu_trigger = 1;

  if(counterWifi >= TRIGGER_100MS){
    counterWifi = 0;
    wifi_trigger = 1;
  }

  if(counterControl >= TRIGGER_CONTROL){
      counterControl = 0;
      control_trigger = 1;

  }
}




void setup() {
  Serial.begin(115200);
  delay(10);

  quadcopter.initializeMotors(27,14,16,17);
  delay(5000);

  timer.initTimer(FREQUENCY, time_count);
  imu.initMPU();

//  wifi.disable_debug();
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

}

void loop() {

  actualVel = quadcopter.getActualVel();
  vel1 = actualVel[0];
  vel2 = actualVel[1];


  Serial.println(vel1);
  Serial.println(vel2);
  

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
      
      //Armazena as velocidades desejadadas de pitch, roll, yaw e throttle desejadas pelo cliente wifi
      setpoints = wifi.getVel();

      //Atualiza o setpoint de velocidade angular de pitch
      pitchVelPid.setSetPoint(setpoints.pitch);
      
      
      //Atualiza potencia dos motores baseado no comando do joystick
      addTrottleTemp = (((float)(setpoints.throttle)/100)+1)*MOTORTHROTTLE;
      quadcopter.setThrottle(addTrottleTemp);

      //Atualiza as variaveis relacionadas ao ganho do controlador de Pitch, quando o usuario insere este comando
      desiredGains.kp = wifi.getPIDGains('p').kp;
      desiredGains.ki = wifi.getPIDGains('p').ki;
      desiredGains.kd = wifi.getPIDGains('p').kd;
      pitchVelPid.setKp(desiredGains.kp);
      pitchVelPid.setKd(desiredGains.kd);
      pitchVelPid.setKi(desiredGains.ki);
      
    }
  
  //Consulta o sinal de pwm enviado pela ultima vez aos motores
    vel1 = quadcopter.getActualVel()[0]; 
    vel2 = quadcopter.getActualVel()[1];
    vel3 = quadcopter.getActualVel()[2]; 
    vel4 = quadcopter.getActualVel()[3];
   
    sprintf(message, "M1:%d\tM2:%d\tM3:%d\tM4:%d || SetPoint: %0.3f || Erro: %0.3f \n Vel P: %0.3f Angulo P: %0.3f Kp: %0.3f Kd: %0.3f Ki: %0.3f \0",vel1,vel2,vel3,vel4, pitchVelPid.getSetPoint(), pitchVelPid.getPreviousError(), imu.getGyroVel().Pitch, imu.getRotations().Pitch, pitchVelPid.getGains().fkp, pitchVelPid.getGains().fkd, pitchVelPid.getGains().fki);
    wifi.sendData(message);
    message[0] = '\0';
    sent_counter++;
  
    if(sent_counter == 5){
      sent_counter = 0;
    }
   
  }

  if(control_trigger){ 
    //Executa a rotina de controle para a velocidade de pitch
        control_trigger = 0;
      //Testa se o drone está com a velocidade angular acima do permitido
        if( (imu.getData().GyY >= 90) || imu.getData().GyY <= -90) {
        //Caso esteja na velocidade angular maxima, deve establizar com velocidade nula no eixo
            pitchVelPid.setSetPoint(0);
            quadcopter.setActualVel(MOTORTHROTTLE,MOTORTHROTTLE,1000,1000);
            delay(5000);   
        } 
        //Atualiza o sinal de saida do controle PID necessário para a velocidade em pitch
        pitchVelPid.pidVelControl(imu.getGyroVel());
        //Envia o sinal de controle para atualizar a potencia dos motores necessária para o movimento de Pitch 
        quadcopter.SingleAxisVelControl(pitchVelPid);
 
  }
delay(100);
}
