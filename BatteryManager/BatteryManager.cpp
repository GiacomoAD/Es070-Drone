#include "BatteryManager.h"

void BatteryManager::initBatteryManager(unsigned char adcPin)     
{
	pinMode(adcPin, INPUT);
	pin = adcPin;

	Serial.println("Battery Manager Initialized!");
	Serial.printf("Reading from pin %d", adcPin);

	return;

}

void BatteryManager::readVoltage()        
{
 	batteryRaw = analogRead(pin);
 	convertValue(batteryRaw);

 	return;

}

float BatteryManager::getVoltage()
{
 	return batteryVoltage;
}

unsigned char BatteryManager::checkBatteryStatus()
{
	if(batteryVoltage <= SAFE_CHARGE)
		return 0;

	else
		return 1;
}

void BatteryManager::convertValue(int voltage)
{

	batteryVoltage = (float)voltage*ADC_CONVERSION;
	batteryVoltage = batteryVoltage*MAX_CHARGE_CONVERSION;

	return;

}
