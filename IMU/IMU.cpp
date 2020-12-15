#include "IMU.h"

void IMU::writeRegMPU(int reg, int val)      //aceita um registro e um valor como parâmetro
{
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                      // escreve o valor no registro
  Wire.endTransmission(true);           // termina a transmissão
}

unsigned char IMU::readRegMPU(unsigned char reg)        // aceita um registro como parâmetro
{
  unsigned char _processedData;
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false);          // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1);        // configura para receber 1 byte do registro escolhido acima
  _processedData = Wire.read();                   // lê o byte e guarda em '_processedData'
  return _processedData;                          //retorna '_processedData'
}

unsigned char IMU::findMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  int _processedData = Wire.endTransmission(true);
  unsigned char ucFound = 0;
 
  if(_processedData == 0)
  {
    if(debbuging_enabled){
      Serial.print("Dispositivo encontrado no endereço: 0x");
      Serial.println(MPU_ADDR, HEX);
    }
    ucFound = 1;
  }
  else
  {
    if(debbuging_enabled)
      Serial.println("Dispositivo não encontrado!");
    ucFound = 0;
  }

  return ucFound;
}

unsigned char IMU::checkMPU()
{

  unsigned char ucCheck = 0;

  if(!findMPU()){
    return ucCheck;
  }
  
  int _processedData = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75
   
  if(_processedData == 104) 
  {
    if(debbuging_enabled)
      Serial.println("MPU6050 Dispositivo respondeu OK! (104)");
 
    _processedData = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B
 
    if(_processedData == 64){
      if(debbuging_enabled)
        Serial.println("MPU6050 em modo SLEEP! (64)");
      ucCheck = 2;
    }
    else{
      if(debbuging_enabled)
        Serial.println("MPU6050 em modo ACTIVE!");
      ucCheck = 1;
    }
  }
  else {
    if(debbuging_enabled)
      Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");
  }

  return ucCheck;

}

void IMU::initMPU()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  setSleepOff();
  setGyroScale();
  setAccelScale();
  checkMPU();
}

void IMU::setSleepOff()
{
  writeRegMPU(PWR_MGMT_1, 0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE
}

void IMU::setGyroScale()
{
  writeRegMPU(GYRO_CONFIG, 0);
}

void IMU::setAccelScale()
{
  writeRegMPU(ACCEL_CONFIG, 0);
}

mpu IMU::readRawMPU()
{  

  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; 

  Wire.beginTransmission(MPU_ADDR);       // inicia comunicação com endereço do MPU6050
  Wire.write(ACCEL_XOUT);                 // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);            // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 14);         // configura para receber 14 bytes começando do registro escolhido acima (0x3B)
 
  AcX = Wire.read() << 8;                 // lê primeiro o byte mais significativo
  AcX |= Wire.read();                     // depois lê o bit menos significativo
  _rawData.AcX = AcX;

  AcY = Wire.read() << 8;
  AcY |= Wire.read();
  _rawData.AcY = AcY;

  AcZ = Wire.read() << 8;
  AcZ |= Wire.read();
  _rawData.AcZ = AcZ;
 
  Tmp = Wire.read() << 8;
  Tmp |= Wire.read();
  _rawData.Tmp = Tmp;
 
  GyX = Wire.read() << 8;
  GyX |= Wire.read();
  _rawData.GyX = GyX;

  GyY = Wire.read() << 8;
  GyY |= Wire.read();
  _rawData.GyY = GyY;

  GyZ = Wire.read() << 8;
  GyZ |= Wire.read(); 
  _rawData.GyZ = GyZ;

  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state);         // pisca LED do NodeMCU a cada leitura do sensor
  return _rawData;                                        
}

void IMU::CalibrateGyro(float X, float Y, float Z){
  calGyX = X;
  calGyY = Y;
  calGyZ = Z;
}

void IMU::CalibrateAcl(float X, float Y){
  calAcX = X;
  calAcY = Y;
}

mpu IMU::getRawData(){
  return _rawData;
}

processedMpu IMU::processMPUData(){

  _processedData.GyX = (float)(_rawData.GyX - calGyX)/131;
  _processedData.GyY = (float)(_rawData.GyY - calGyY)/131;
  _processedData.GyZ = (float)(_rawData.GyZ - calGyZ)/131;

  _processedData.AcX = (float)(_rawData.AcX - calAcX)/16384;
  _processedData.AcY = (float)(_rawData.AcY- calAcY)/16384;
  _processedData.AcZ = (float)_rawData.AcZ/16384;

  _processedData.Tmp = (float)_rawData.Tmp;

  processAngles(_processedData);

  return _processedData;
}

void IMU::processAngles(processedMpu dados){

  unsigned long aux = micros();
  float dt = (float)(aux - _lastTimestamp)/1000000;
  float x2, y2, z2, result;
  _lastTimestamp = aux;

  //Serial.println(dt);

  _ang.GyPitch = _procAng.Pitch;
  _ang.GyRoll  = _procAng.Roll;
  _ang.GyYaw   = _procAng.Yaw;
  
  _ang.GyPitch  += _processedData.GyX*dt; //MPU eixo X

  _ang.GyPitch  += _ang.GyRoll*sin(_ang.GyYaw*dt*DEG_2_RAD);

  _ang.GyRoll   += _processedData.GyY*dt; //MPU eixo Y

  _ang.GyRoll   -= _ang.GyPitch*sin(_ang.GyYaw*dt*DEG_2_RAD);

  _ang.GyYaw    += _processedData.GyZ*dt; //MPU eixo Z

  
/*
  Serial.printf("Acelerometer INPUT:\t %f\t%f\t%f\n",_processedData.AcX, _processedData.AcY, _processedData.AcZ);
*/
  x2 = _processedData.AcX*_processedData.AcX;
  y2 = _processedData.AcY*_processedData.AcY;
  z2 = _processedData.AcZ*_processedData.AcZ;

  //Pitch
  result = sqrt(x2+z2);
  _ang.AclPitch = -1*atan2(-1*_processedData.AcY, result)*RAD_2_DEG;

  //Roll
  _ang.AclRoll  = -1*atan2(_processedData.AcX, _processedData.AcZ)*RAD_2_DEG;

  filterMPUData();

  if(debbuging_enabled)
    Serial.printf("Gyro:%f\tAccl:%f\tCompl:%f\n",_ang.GyRoll, _ang.AclRoll, _procAng.Roll);

/*
  Serial.printf("ANGULOS GIROSCOPIO:\nROLL:%f\tPITCH:%f\t%YAW:%f\n",_ang.GyRoll, _ang.GyPitch, _ang.GyYaw);
  Serial.printf("ANGULOS ACELEROMETRO:\nROLL:%f\tPITCH:%f\n",_ang.AclRoll, _ang.AclPitch);
  Serial.printf("ANGULOS COMPLEMENTAR:\nROLL:%f\tPITCH:%f\t%YAW:%f\n\n", _procAng.Roll, _procAng.Pitch, _procAng.Yaw);
*/
}

void IMU::filterMPUData( ){

  _procAng.Roll   =   CF_GY*_ang.GyRoll + CF_AC*_ang.AclRoll;
  _procAng.Pitch  =   CF_GY*_ang.GyPitch + CF_AC*_ang.AclPitch;
  _procAng.Yaw    =   _ang.GyYaw;


}

void IMU::update(){
  readRawMPU();
  processMPUData();
}

processedMpu IMU::getData(){

  return _processedData;

}

angles IMU::getRawAngles(){
  return _ang;
}

processedAngles IMU::getRotations(){
  return _procAng;
}

void IMU::enableDebug(){
  debbuging_enabled = 1;
}

void IMU::disableDebug(){
  debbuging_enabled = 0;
}