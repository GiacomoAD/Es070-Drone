/*******************************************************************
 * Descrição: Arquivo header para implementação das funções de  
 * interface de controle de velocidade dos motores do Drone
 * Autores: Gustavo L. Fernandes e Giácomo A. Dollevedo
 * Ultima Atualização: 03/01/2021
********************************************************************/

#ifndef ThrottleControl_h
#define ThrottleControl_h

#include "FlightControl.h"
#include "Arduino.h"
#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>

#define MAXTHROTTLE 1500
#define MOTORTHROTTLE 1060 //sinal em que os motores começam a atuar
#define PINMOTOR1 13
#define PINMOTOR2 27
#define PINMOTOR3 14
#define PINMOTOR4 12




class ThrottleControl
{

    private:

    Servo _m1;
    Servo _m2;
    Servo _m3;
    Servo _m4;
    int _throttle;
    int* _actualVel;
    
  public:
/* ******************************************************************************** */
/* Nome do metodo:          ThrottleControl                                         */
/* Descrição:               Método construtor do objeto, não faz nada além de       */
/*                          associar 4 objetos do tipo Servo que serão utilizados   */
/*                                                                                  */
/* Parametros de entrada:    Nenhum (Vazio)                                         */
/*                                                                                  */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */
    ThrottleControl();

/* ******************************************************************************** */
/* Nome do metodo:          initializeMotors                                        */
/* Descrição:              Inicia cada um dos motores criados, atrelando a uma saída*/
/*                         PWM do ESP32 e setando os limites max e min que define   */
/*                         também a inicialização dos 4 ESC                         */
/*                                                                                  */
/* Parametros de entrada: int pinMotor1, int pinMotor2, int pinMotor3, int pinMotor4*/
/*                        Pinos PWM que serão conectados os sinais de controle      */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */

    void initializeMotors(int pinMotor1, int pinMotor2, int pinMotor3, int pinMotor4);

/* ******************************************************************************** */
/* Nome do metodo:         setActualVel                                             */
/* Descrição:              Define as velocidades que serão atingidas pelos 4 motores*/
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: int desiredVel1, int desiredVel2, int desiredVel3,        */
/*                        int desiredVel4 , que são as velocidades desejadas        */
/*                        aceita valores entre 1000 e 2000 mas limita em 1500       */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */    
    void setActualVel(int desiredVel1, int desiredVel2, int desiredVel3, int desiredVel4);
/* ******************************************************************************** */
/* Nome do metodo:         getActualVel                                             */
/* Descrição:              Consulta o sinal pwm definido nos 4 motores              */
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: Nenhum (Vazio)                                            */
/*                                                                                  */
/* Parametros de saida:   int* array com os 4 valores de velocidades lidos          */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */  
    int* getActualVel(); 

/* ******************************************************************************** */
/* Nome do metodo:         gtestMotors                                              */
/* Descrição:              Testa se os motores estão funcinoando e respondendo      */
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: Nenhum (Vazio)                                            */
/*                                                                                  */
/* Parametros de saida:   boolean  (1 Funcionando) (0 Com problemas)                */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */
    void testMotors();

/* ******************************************************************************** */
/* Nome do metodo:         Control                                                  */
/* Descrição:              Distribuia a velocidade controlada para os 4 motores     */
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: FlightControl pidRoll, FlightControl pidPitch,            */
/*                        IMU imu  , que são os objetos  do controlador             */
/*                        implementado para cada eixo e sensor de inercia           */
/*                                                                                  */
/* Parametros de saida:  Vazio (Nenhum)                                             */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */    
    void Control(FlightControl pidRoll, FlightControl pidPitch, IMU imu);

/* ******************************************************************************** */
/* Nome do metodo:         SingleAxisVelControl                                     */
/* Descrição:              Distribui  a velocidade controlada para os 2 motores     */
/*                         de modo a contrlar apenas um eixo de movimento           */
/*                                                                                  */
/* Parametros de entrada: FlightControl pidRoll que é objeto do eixo roll de        */
/*                        controle            ,                                     */
/*                                                                                  */
/*                                                                                  */
/* Parametros de saida:  Vazio (Nenhum)                                             */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */    
    void SingleAxisVelControl(FlightControl pidRoll);

/* ******************************************************************************** */
/* Nome do metodo:         getThrottle                                              */
/* Descrição:              Consulta a velocidade (pwm) base atual dos motores       */
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: Vazio (Nenhum)                                            */
/*                                                                                  */
/*                                                                                  */
/*                                                                                  */
/* Parametros de saida:  _throttle (int)                                            */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */
    int getThrottle(); 

/* ******************************************************************************** */
/* Nome do metodo:         setThrottle                                              */
/* Descrição:              Define nova velocidade (pwm) base atual dos motores      */
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: throttleDesired (int)                                     */
/*                                                                                  */
/*                                                                                  */
/*                                                                                  */
/* Parametros de saida:  Vazio (Nenhum )                                            */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */

    void setThrottle(int throttleDesired);

};

#endif
