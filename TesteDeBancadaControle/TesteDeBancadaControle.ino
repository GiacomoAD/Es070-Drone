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

#define FREQUENCY 1000
#define TRIGGER_100MS FREQUENCY/10
#define TRIGGER_1S FREQUENCY
#define TRIGGER_CONTROL FREQUENCY/250

int         sent_counter  =   0;
char*       message       = (char*)calloc(1024, sizeof(char));
String      command       = "";

//Variaveis para contabilizar quanto falta para o trigger de cada tarefa
volatile unsigned char counterWifi = 0;
volatile unsigned char counterControl = 0;
int counterTest = 0;

//Variaveis de Trigger de cada tarefa
volatile unsigned char control_trigger = 0;
volatile unsigned char wifi_trigger = 0;
volatile unsigned char imu_trigger = 0;
volatile unsigned char test_trigger = 0;

int* actualVel = (int*) calloc(3, sizeof(int));
int vel1, vel2, vel3, vel4;
pidGains desiredGains;



DroneTimer timer;
DroneWiFi wifi;
IMU imu;
FlightControl rollVelPid(0.0f, 0.0f, 0.0f , 'r');
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



rotVel setpoints;
float addTrottleTemp;

void IRAM_ATTR time_count(){

  counterControl++;
  counterWifi++;
  counterTest++;

  if(imu_trigger == 0){
    imu_trigger = 1;
    control_trigger = 1; 
  }

  if(counterWifi >= TRIGGER_100MS){
    counterWifi = 0;
    wifi_trigger = 1;
  }
}


void setup() {
  Serial.begin(115200);
  delay(10);

  quadcopter.initializeMotors(13,12,16,17);
  delay(2000);

  timer.initTimer(FREQUENCY, time_count);
  imu.initMPU();
  imu.disableYawComp();
  imu.disableDebug();

  wifi.disable_debug();
  wifi.initWiFi(SSID_GUS, PASS_GUS, IP_GIA, PORT);

  setpoints = wifi.getVel();
  delay(10);
  timer.enableTimer();

  long tempCal = micros();
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
    yield();
  }

  if(calibrar < 2500){    
    imu.CalibrateGyro(calx/calibrar, caly/calibrar, calz/calibrar);
    imu.CalibrateAcl(calAx/calibrar, calAy/calibrar);
    calibrar = 3000;
  }

    Serial.print("\n Saiu irmão");
    rollVelPid.setSetPoint(0);
    
}

void loop() { 

  actualVel = quadcopter.getActualVel();
  vel1 = actualVel[0];
  vel2 = actualVel[1];


  if(wifi_trigger){
    wifi_trigger = 0;

    rotations = imu.getRotations();
    
    if(sent_counter == 0){
      command = wifi.receiveData();
      wifi.processComm((String)command);
      
      //Armazena as velocidades desejadadas de roll, roll, yaw e throttle desejadas pelo cliente wifi
      setpoints = wifi.getVel();    
      
      //Atualiza potencia dos motores baseado no comando do joystick
      addTrottleTemp = (((float)(setpoints.throttle)/100)+1)*MOTORTHROTTLE;
      quadcopter.setThrottle(addTrottleTemp);


        //Atualiza as variaveis relacionadas ao ganho do controlador de Roll, quando o usuario insere este comando
        desiredGains.kp = wifi.getPIDGains('p').kp;
        desiredGains.ki = wifi.getPIDGains('p').ki;
        desiredGains.kd = wifi.getPIDGains('p').kd;
        rollVelPid.setKp(desiredGains.kp);
        rollVelPid.setKd(desiredGains.kd);
        rollVelPid.setKi(desiredGains.ki);

      //Atualiza o setpoint de velocidade angular de roll
        rollVelPid.setSetPoint(setpoints.roll);
    }
  
  //Consulta o sinal de pwm enviado pela ultima vez aos motores
    vel1 = quadcopter.getActualVel()[0]; 
    vel2 = quadcopter.getActualVel()[1];
    vel3 = quadcopter.getActualVel()[2]; 
    vel4 = quadcopter.getActualVel()[3];
   
    sprintf(message, "M1:%d\tM2:%d\tM3:%d\tM4:%d || SetPoint: %0.3f || Erro: %0.3f \n Vel P: %0.3f Angulo P: %0.3f Kp: %0.3f Kd: %0.3f Ki: %0.3f \0",vel1,vel2,vel3,vel4, rollVelPid.getSetPoint(), rollVelPid.getPreviousError(), imu.getRollVel(), imu.getRotations().Roll, rollVelPid.getGains().fkp, rollVelPid.getGains().fkd, rollVelPid.getGains().fki);
    wifi.sendData(message);
    message[0] = '\0';
    sent_counter++;
  
    if(sent_counter == 5){
      sent_counter = 0;
    }
  }

  if(imu_trigger){ 
    control_trigger = 0;
    imu_trigger = 0;
    //Atualiza os dados de inercia do drone
    imu.update();
    //Executa a rotina de controle para a velocidade de roll
        
      //Testa se o drone está com a velocidade angular acima do permitido
        if( (imu.getData().GyY >= 90) || imu.getData().GyY <= -90) {
        //Caso esteja na velocidade angular maxima, deve establizar com velocidade nula no eixo
            rollVelPid.setSetPoint(0);
            quadcopter.setActualVel(MOTORTHROTTLE,MOTORTHROTTLE,1000,1000);
            delay(5000);   
        } 
        //Atualiza o sinal de saida do controle PID necessário para a velocidade em roll
        rollVelPid.pidVelControl(imu);
        //Envia o sinal de controle para atualizar a potencia dos motores necessária para o movimento de Roll 
        quadcopter.SingleAxisVelControl(rollVelPid);
 

      //Se estiver dentro da rotina de teste, deve printar os valores de velocidade atual e de angulo

      Serial.print(imu.getRollVel());
      Serial.print(" ");
      Serial.print(imu.getRotations().Roll);
      Serial.print("\n");
  }
}
