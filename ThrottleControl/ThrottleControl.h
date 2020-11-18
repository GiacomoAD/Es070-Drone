/*******************************************************************
 * Descrição: Arquivo header para implementação das funções de  
 * interface de controle de velocidade dos motores do Drone
 * Autores: Gustavo L. Fernandes e Giácomo A. Dollevedo
 * Ultima Atualização: 18/11/2020
********************************************************************/

#ifndef ThrottleControl_h
#define ThrottleControl_h

#include "Arduino.h"
#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>



class ThrottleControl
{

    private:

    Servo _m1;
    Servo _m2;
    Servo _m3;
    Servo _m4;
    
  public:
    ThrottleControl();
    void initializeMotors(int pinMotor1, int pinMotor2, int pinMotor3, int pinMotor4);
    void setTrottle(int desiredVel1, int desiredVel2, int desiredVel3, int desiredVel4);
    int* getTrottle(); 
    boolean testMotors();
    void Control(FlightControl pidRoll, FlightControl pidPitch, FlightControl pidYaw );

};

#endif
