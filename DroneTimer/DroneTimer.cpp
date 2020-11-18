#include "DroneTimer.h"

void DroneTimer::initTimer(int frequency, void (*fn)(void))     
{
	timer0 = timerBegin(0, 80, true);
  	timerAttachInterrupt(timer0, fn, true);
  	timerAlarmWrite(timer0, 1000000/frequency, true);
  	Serial.println("Timer Interrupt Attached!");
}

void DroneTimer::enableTimer()        
{
 	timerAlarmEnable(timer0);
}

void DroneTimer::disableTimer()
{
 	timerAlarmDisable(timer0);
}

void DroneTimer::setFrequency(int frequency)
{
	timerFreq = frequency;
}

int DroneTimer::getFrequency()
{
	return timerFreq;
}
