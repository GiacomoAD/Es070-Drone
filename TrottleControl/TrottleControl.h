/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef TrottleControl_h
#define TrottleControl_h

#include "Arduino.h"
#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>

class TrottleControl
{

  private:
  
    Servo _m1;
    Servo _m2;
    Servo _m3;
    Servo _m4;
    
  public:
    TrottleControl();
    void initializeMotors(int pinMotor1, int pinMotor2, int pinMotor3, int pinMotor4);
    void setTrottle(int desiredVel1, int desiredVel2, int desiredVel3, int desiredVel4);

};

#endif
