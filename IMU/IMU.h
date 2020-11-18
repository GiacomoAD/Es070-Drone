/*
 IMU.h
*/
 
#ifndef IMU_h
#define IMU_h

//Register Map MPU-6050 https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
#include <Wire.h>
#include <Arduino.h>
#define I2C_SDA 21
#define I2C_SCL 22


#define MPU_ADDR       0x68 // definição do endereço do sensor MPU6050
#define WHO_AM_I       0x75 // registro de identificação do dispositivo
#define PWR_MGMT_1     0x6B // registro de configuração do gerenciamento de energia
#define GYRO_CONFIG    0x1B // registro de configuração do giroscópio
#define ACCEL_CONFIG   0x1C // registro de configuração do acelerômetro
#define ACCEL_XOUT     0x3B // registro de leitura do eixo X do acelerômetro
#define LED_BUILTIN    2    // LED do DevKit v1

typedef struct mpu
 {
  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
 };

typedef struct processedMpu
{
 float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
};

class IMU
{
public:
 void initMPU();
 mpu readRawMPU();
 processedMpu processMPUData(mpu dados);
 processedMpu filterMPUData(processedMpu dados);
 processedMpu getData();
 
 
private:
  void writeRegMPU(int reg, int val);
  unsigned char readRegMPU(unsigned char reg);
  unsigned char findMPU();
  unsigned char checkMPU();
  
  void setSleepOff();
  void setGyroScale();
  void setAccelScale();
  
  processedMpu data; 

  int calGyX = 0;
  int calGyY = 0;
  int calGyZ = 0;
  
  int calAcX = 0;
  int calAcY = 0;
  int calAcZ = 0;
  
  
  unsigned char led_state = 0;
  
};
 
#endif
