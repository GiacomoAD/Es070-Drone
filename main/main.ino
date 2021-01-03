/*****************************************************************************\
 * Descrição: Main para testes em bancada de calibração do controlador PID   *
 * do Drone                                                                  *
 * Autores: Gustavo L. Fernandes e Giácomo A. Dollevedo                      *  
 * Data de Criacao:     03jan2021                                            *
 * Ultima Atualização:  03jan2021                                            *
******************************************************************************/

#include "DroneWiFi.h"
#include "IMU.h"
#include "DroneTimer.h"
#include "FlightControl.h"
#include "ThrottleControl.h"

//Frequencia Base do Timer (1kHz)
#define FREQUENCY 1000
#define TRIGGER_100MS   FREQUENCY/10
#define TRIGGER_CONTROL FREQUENCY/250

//Contador de mensagens enviados por WiFi
int         sent_counter  =   0;
//Alocacao de memoria para mensagem que sera enviada por WiFi
char*       message       = (char*)calloc(1024, sizeof(char));
//Comando recebido via WiFi
String      command       = "";

//Variaveis para contabilizar quanto falta para o trigger de cada tarefa
volatile unsigned char counterWifi = 0;
volatile unsigned char counterControl = 0;

//Variaveis de Trigger de cada tarefa
unsigned char control_trigger = 0;
unsigned char wifi_trigger = 0;
unsigned char imu_trigger = 0;

/*Objeto do tipo Timer para controlar interrupcoes*/
DroneTimer timer;
/*Objeto do tipo WiFI para realizar a transferencia de dados*/
DroneWiFi wifi;
/*Objeto do tipo IMU para leitura e calculo dos parametros sensoriados*/
IMU imu;
/*Objeto do tipo FlightControl para realizar o controle de velocidade de Pitch*/
FlightControl pitchVelPid(0.5f, 0.8f, 0.9f , 'p');
/*Objeto do tipo FlightControl para realizar o controle de velocidade de Roll*/
FlightControl rollVelPid(0.0 ,0.0, 0.0, 'r');
/*Objeto do tipo ThrottleControl para realizar o controle dos motores*/
ThrottleControl quadcopter;

mpu rawData;
processedAngles rotations;

int calibrar = 0;
float calx  = 0;
float caly  = 0;
float calz  = 0;



rotVel setpoints;
float addThrottleTemp;

/*Estado do sistema*/
/*0 == modo de testes | 1 == modo joystick*/
unsigned char estado_sistema = 0;

/*Structs com os ganhos Kp, Ki e Kd de Roll e Pitch*/
pidGains ganhosRoll;
pidGains ganhosPitch;

/*Struct com os parametros de monitoramento do drone*/
droneParams parametrosDrone;

/*Vetor que contem as velocidades individuais de cada motor*/
int* actualVel = (int*)calloc(4, sizeof(int));


/*time_count*/
/*seta as flags para controle de tempo do programa*/
void IRAM_ATTR time_count(){

  counterControl++;
  counterWifi++;
 
  //WiFi eh disparado a cada 100ms
  if(counterWifi >= TRIGGER_100MS){
    counterWifi = 0;
    wifi_trigger = 1;
  }

  //Atualizacao da IMU e rotina de controle a cada 4ms
  if(counterControl >= TRIGGER_CONTROL){
      counterControl = 0;
      control_trigger = 1;   
      imu_trigger = 1;
  }
  
}



/*ROTINA DE INICIALIZACAO DO SISTEMA*/
void setup() {

  Serial.begin(115200); //COMENTAR, NAO VAMOS USAR SERIAL NA MAIN!
  delay(10);

  /*Inicializacao dos motores*/
  quadcopter.initializeMotors(27,14,16,17);
  delay(2000);

  /*Inicializacao do timer*/
  timer.initTimer(FREQUENCY, time_count);
  imu.initMPU();

//  imu.disableDebug();
//  wifi.disable_debug();
  /*Inicializacao do WiFi*/
  wifi.initWiFi(SSID_GUS, PASS_GUS, IP_GIA, PORT);

  setpoints = wifi.getVel();
  delay(10);

  /*Habilita as interrupcoes do timer*/
  timer.enableTimer();

  /*Calibracao da IMU*/
  while(calibrar < 2000){
    if(imu_trigger){
  
      imu_trigger = 0;
      imu.readRawMPU();
      rawData = imu.getRawData();

      calx  += rawData.GyX;
      caly  += rawData.GyY;
      calz  += rawData.GyZ;
      
      calibrar++;
    }
  }
  imu.CalibrateGyro(calx/calibrar, caly/calibrar, calz/calibrar);
  
  pitchVelPid.setSetPoint(0);
  rollVelPid.setSetPoint(0);
    
}



void loop() { 

  /*Rotina de execucao das operacoes relacionadas ao WiFi*/
  /*Ocorre a cada 100ms*/
  if(wifi_trigger){
    /*Reseta flag do WiFi*/
    wifi_trigger = 0;

    /*Se eh a primeira mensagem de 5, recebe um comando do servidor*/
    if(sent_counter == 0){
      /*Recebe o comando do Servidor*/
      command = wifi.receiveData();
      /*Realiza a rotina de tratamento deste comando*/
      wifi.processComm((String)command);

      /*Troca a flag do sistema para operar no modo Joystick*/
      if(command == "#J"){
        estado_sistema = 1;
      }

      /*Troca a flag do sistema para operar no modo teste*/
      else if(command == "s"){
        estado_sistema = 0;
      }

      /*Aqui ocorrem as alteracoes dos parametros de acordo com o input do usuario*/
      switch(estado_sistema){
      /*Atualiza as variaveis de acordo com os comandos de teste*/
      case 0:
     
        ganhosRoll  = wifi.getPIDGains('r');
        ganhosPitch = wifi.getPIDGains('p');

        parametrosDrone = wifi.getParams();

        break;

      /*Atualiza as variaveis de acordo com o Joystick*/
      case 1:
        /*Valores dos setpoints pelo joystick*/
        setpoints = wifi.getVel();
        break;

      default:
        break;
      }
    }
  
    switch(estado_sistema){
      /*Atualiza as variaveis de acordo com os comandos de teste*/
      case 0:
        /*Atualizacoes de Kp, Ki e Kd para Pitch*/
        pitchVelPid.setKp(ganhosPitch.kp);
        pitchVelPid.setKd(ganhosPitch.kd);
        pitchVelPid.setKi(ganhosPitch.ki);
        /*Atualizacoes de Kp, Ki e Kd para Roll*/
        rollVelPid.setKp(ganhosRoll.kp);
        rollVelPid.setKd(ganhosRoll.kd);
        rollVelPid.setKi(ganhosRoll.ki);

        /*Atualiza velocidades dos motores baseado no input do usuario*/
        quadcopter.setActualVel(parametrosDrone.M1,parametrosDrone.M2,parametrosDrone.M3,parametrosDrone.M4);

        actualVel = quadcopter.getActualVel();

        /*Escreve a mensagem que sera enviada ao Servidor*/
        sprintf(message, "M1:%d\tM2:%d\tM3:%d\tM4:%d || SetPoint: %0.3f || Erro: %0.3f \n Vel P: %0.3f Angulo P: %0.3f Kp: %0.3f Kd: %0.3f Ki: %0.3f \0", actualVel[0], actualVel[1], actualVel[2], actualVel[3], pitchVelPid.getSetPoint(), pitchVelPid.getPreviousError(), imu.getPitchVel(), imu.getRotations().Pitch, pitchVelPid.getGains().fkp, pitchVelPid.getGains().fkd, pitchVelPid.getGains().fki);
     

        break;

      /*Atualiza as variaveis de acordo com o Joystick*/
      case 1:
        /*Atualiza potencia dos motores baseado no comando do joystick*/
        addThrottleTemp = (((float)(setpoints.throttle)/100)+1)*MOTORTHROTTLE;
        quadcopter.setThrottle(addThrottleTemp);

        /*Atualiza o setpoint de velocidade angular de pitch*/
        pitchVelPid.setSetPoint(setpoints.pitch);

        /*Atualiza o setpoint de velocidade angular de pitch*/
        rollVelPid.setSetPoint(setpoints.roll);


        /*Escreve a mensagem que sera enviada ao Servidor*/
        sprintf(message, "SetPoint Roll:\t%0.2f | SetPoint Pitch:\t%0.2f\t Potencias M1:%d\tM2:%d\nAngulo Roll:\t%0.2f | Angulo Pitch:\t%0.2f\tPotencias M3:$d\tM4:%d\0", rollVelPid.getSetPoint(), pitchVelPid.getSetPoint(), actualVel[0], actualVel[1], rotations.Roll, rotations.Pitch,actualVel[2], actualVel[3]);

        break;

      default:
        break;
    }
   
    /*Envia a mensagem*/
    wifi.sendData(message);

    /*Escreve o caractere finalizador para resetar a mensagem*/
    message[0] = '\0';

    /*Aumenta contador de mensagens consecutivas enviadas*/
    sent_counter++;
  
    /*Com 5 mensagens enviadas consecutivas, reseta o contador*/
    if(sent_counter == 5){
      sent_counter = 0;
    }
  }

  /*Rotina de execucao de atualizacao da IMU e de controle*/
  /*Ocorre a cada 4ms*/
  if(control_trigger){ 
    control_trigger = 0;
    /*Atualiza os dados de inercia do drone*/
    imu.update();

    /*Angulos de Pitch e Roll apos filtro complementar*/
    rotations = imu.getRotations();


    /**********************************/
    /*INSERIR ROTINA DE SEGURANCA AQUI*/
    /**********************************/
    
    /*Atualiza o sinal de saida do controle PID necessário para a velocidade em roll*/
    rollVelPid.pidVelControl(imu);
    /*Atualiza o sinal de saida do controle PID necessário para a velocidade em pitch*/
    pitchVelPid.pidVelControl(imu);

    /*Envia o sinal de controle para atualizar a potencia dos motores necessária para o movimento de Roll e Pitch*/
    quadcopter.Control(rollVelPid, pitchVelPid);
    
    /*Le a velocidade atual do drone*/
    actualVel = quadcopter.getActualVel();

    /*Atualiza os parametros de velocidade do drone*/
    parametrosDrone.M1 = actualVel[0];
    parametrosDrone.M2 = actualVel[1];
    parametrosDrone.M3 = actualVel[2];
    parametrosDrone.M4 = actualVel[3];
    wifi.setParams(parametrosDrone);
 
  }
}
