/*******************************************************************
 * Descrição: Arquivo header para implementação das funções de  
 * interface de controle de velocidade dos motores do Drone
 * Autores: Gustavo L. Fernandes e Giácomo A. Dollevedo
 * Ultima Atualização: 18/11/2020
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
    
  public:
    ThrottleControl();
    void initializeMotors(int pinMotor1, int pinMotor2, int pinMotor3, int pinMotor4);
    void setActualVel(int desiredVel1, int desiredVel2, int desiredVel3, int desiredVel4);
    int* getActualVel(); 
    boolean testMotors();
    void Control(FlightControl pidRoll, FlightControl pidPitch, FlightControl pidYaw );
    void SingleAxisVelControl(FlightControl pidRoll);
    int getThrottle(); 
    void setThrottle(int throttleDesired);

};

#endif
