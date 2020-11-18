/*****************************************************
 * Descrição: Arquivo header das funções responsáveis 
 * pelo controle de voo do drone. 
 * Autores: Gustavo L. Fernandes e Giácomo A. Dollevedo
 * Ultima Atualização: 14/11/2020
*****************************************************/
#ifndef FlightControl_h
#define FlightControl_h

#include "Arduino.h"
#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>

typedef struct kPID
{
float fkp, fki, fkd;
}; 

class FlightControlControl
{

  private:
    kPID _gains;
    static float _fsetPoint;
    float _fpidCalculated;
    float _fimem; 
    float _ferror_previous;
    char  _axis;
    
    
  public:
    FlightControl(char Axis);
    FlightControl(float fkp, float fki, float fkd, char Axis);
    float getPID_Calculated()
    kPID getGains();
    void setKp(float kp);
    void setKd(float kd);
    void setKi(float ki);
    void pidControl(IMU imu);

};





#endif