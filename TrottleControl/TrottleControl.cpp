#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>
#include "TrottleControl.h"


TrottleControl::TrottleControl() 
{ 
}

void TrottleControl::initializeMotors(int pinMotor1, int pinMotor2, int pinMotor3, int pinMotor4)
{

    _m1.attach(pinMotor1,1000, 2000); 
    _m2.attach(pinMotor2,1000, 2000);
    _m3.attach(pinMotor3,1000, 2000);
    _m4.attach(pinMotor4,1000, 2000);
    
    _m1.write(1000);
    _m2.write(1000);
    delay(2000);
    _m1.write(2000);
    _m2.write(2000);

    _m3.write(1000);
    _m4.write(1000);
    delay(2000);
    _m3.write(2000);
    _m4.write(2000);
}

void TrottleControl::setTrottle(int desiredVel1, int desiredVel2, int desiredVel3, int desiredVel4)
{
  if((desiredVel1 < 1500) & (desiredVel2 < 1500) & (desiredVel3 < 1500) & (desiredVel4 < 1500)){
    _m1.write(desiredVel1); 
    _m2.write(desiredVel2); 
    _m3.write(desiredVel3);
    _m4.write(desiredVel4);
  }
}
