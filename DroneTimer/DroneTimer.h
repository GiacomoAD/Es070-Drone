/*
 DroneTimer.h
*/
 
#ifndef DroneTimer_h
#define DroneTimer_h


#include <Arduino.h>





class DroneTimer
{

public:
  void initTimer(int frequency, void (*fn)(void));
  void enableTimer();
  void disableTimer();
  void setFrequency(int frequency);
  int getFrequency();
 
private:
  /*Hardware timer for system loop time management*/
  hw_timer_t * timer0 = NULL;
  
  int timerFreq = 10;

};
 
#endif
