/*
 BatteryManager.h
*/
 
#ifndef BatteryManager_h
#define BatteryManager_h

#include <Arduino.h>

#define BATTERY_ADC 34
#define MAX_CHARGE_V 12
#define MAX_CHARGE_CONVERSION MAX_CHARGE_V/3.3
#define ADC_CONVERSION 3.3/4095
#define SAFE_CHARGE 10.5

class BatteryManager
{

public:
  void initBatteryManager(unsigned char adcPin);
  void readVoltage();
  float getVoltage();
  unsigned char checkBatteryStatus();
 
private:
  
  unsigned char pin = 0;
  int batteryRaw = 0;
  float batteryVoltage = 0;

  void convertValue(int voltage);

};
 
#endif
