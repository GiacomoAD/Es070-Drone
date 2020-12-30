/*****************************************************
 * Descrição: Arquivo header das funções responsáveis 
 * pelo controle de voo do drone. 
 * Autores: Gustavo L. Fernandes e Giácomo A. Dollevedo
 * Ultima Atualização: 14/11/2020
*****************************************************/
#ifndef FlightControl_h
#define FlightControl_h

#include "Arduino.h"
#include "IMU.h"
#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>

#define PIDMAX 80

typedef struct kPID
{
float fkp, fki, fkd;
}; 

class FlightControl
{

  private:
    kPID _gains;
    float _fsetPoint;
    float _fpidCalculated;
    float _fimem; 
    float _ferror_previous;
    char  _axis;
    float _setPoint;
    
    
  public:
    FlightControl(char Axis);
    FlightControl(float fkp, float fki, float fkd, char Axis);
    float getPID_Calculated();
    kPID getGains();
    void setKp(float kp);
    void setKd(float kd);
    void setKi(float ki);
    void pidControl(processedMpu imu);
    void setSetPoint(float newSetPoint);
    float getSetPoint();
    float getAccError();
    float getPreviousError();

};





#endif
