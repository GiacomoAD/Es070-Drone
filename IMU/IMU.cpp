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
  unsigned char data;
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false);          // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1);        // configura para receber 1 byte do registro escolhido acima
  data = Wire.read();                   // lê o byte e guarda em 'data'
  return data;                          //retorna 'data'
}

unsigned char IMU::findMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  int data = Wire.endTransmission(true);
  unsigned char ucFound = 0;
 
  if(data == 0)
  {
    Serial.print("Dispositivo encontrado no endereço: 0x");
    Serial.println(MPU_ADDR, HEX);
    ucFound = 1;
  }
  else
  {
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
  
  int data = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75
   
  if(data == 104) 
  {
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");
 
    data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B
 
    if(data == 64){
      Serial.println("MPU6050 em modo SLEEP! (64)");
      ucCheck = 2;
    }
    else{
      Serial.println("MPU6050 em modo ACTIVE!");
      ucCheck = 1;
    }
  }
  else {
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
  mpu mpuData;

  Wire.beginTransmission(MPU_ADDR);       // inicia comunicação com endereço do MPU6050
  Wire.write(ACCEL_XOUT);                 // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);            // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 14);         // configura para receber 14 bytes começando do registro escolhido acima (0x3B)
 
  AcX = Wire.read() << 8;                 // lê primeiro o byte mais significativo
  AcX |= Wire.read();                     // depois lê o bit menos significativo
  mpuData.AcX = AcX;

  AcY = Wire.read() << 8;
  AcY |= Wire.read();
  mpuData.AcY = AcY;

  AcZ = Wire.read() << 8;
  AcZ |= Wire.read();
  mpuData.AcZ = AcZ;
 
  Tmp = Wire.read() << 8;
  Tmp |= Wire.read();
  mpuData.Tmp = Tmp;
 
  GyX = Wire.read() << 8;
  GyX |= Wire.read();
  mpuData.GyX = GyX;

  GyY = Wire.read() << 8;
  GyY |= Wire.read();
  mpuData.GyY = GyY;

  GyZ = Wire.read() << 8;
  GyZ |= Wire.read(); 
  mpuData.GyZ = GyZ;

  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state);         // pisca LED do NodeMCU a cada leitura do sensor
  return mpuData;                                        
}

processedMpu IMU::processMPUData(mpu dados){
  processedMpu aux;

  aux.GyX = (float)(dados.GyX - calGyX)/131;
  aux.GyY = (float)(dados.GyY - calGyY)/131;
  aux.GyZ = (float)(dados.GyZ - calGyZ)/131;

  aux.AcX = (float)dados.AcX/16384;
  aux.AcY = (float)dados.AcY/16384;
  aux.AcZ = (float)dados.AcZ/16384;

  aux.Tmp = (float)dados.Tmp;

  return aux;
}

processedMpu IMU::filterMPUData(processedMpu dados){

  //Implementar filtro complementar

  data = dados;

}

processedMpu IMU::getData(){

  return data;

}
