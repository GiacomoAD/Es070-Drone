  /*
 IMU.h
*/
 
#ifndef IMU_h
#define IMU_h

//Register Map MPU-6050 https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

#define I2C_SDA 21
#define I2C_SCL 22


#define MPU_ADDR       0x68 // definição do endereço do sensor MPU6050
#define WHO_AM_I       0x75 // registro de identificação do dispositivo
#define PWR_MGMT_1     0x6B // registro de configuração do gerenciamento de energia
#define GYRO_CONFIG    0x1B // registro de configuração do giroscópio
#define ACCEL_CONFIG   0x1C // registro de configuração do acelerômetro
#define ACCEL_XOUT     0x3B // registro de leitura do eixo X do acelerômetro
#define LED_BUILTIN    2    // LED do DevKit v1

#define CF_GY          0.90
#define CF_AC          0.10

#define RAD_2_DEG      57.2958 
#define DEG_2_RAD      0.01745

typedef struct mpu
 {
  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
 };

typedef struct processedMpu
{
 float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
};

typedef struct angles
{
  float GyRoll = 0;
  float GyPitch = 0;
  float GyYaw = 0;
  float AclRoll = 0;
  float AclPitch = 0;
  float AclYaw = 0;
};

typedef struct processedAngles
{
  float Roll = 0;
  float Pitch = 0;
  float Yaw = 0;
};

class IMU
{
public:
 void initMPU();
 mpu readRawMPU();
 processedMpu getData();
 mpu getRawData();
 angles getRawAngles();
 processedAngles getRotations();
 void CalibrateGyro(float X, float Y, float Z);
 void CalibrateAcl(float X, float Y);
 void update();
 void enableDebug();
 void disableDebug();
 
private:
  void writeRegMPU(int reg, int val);
  unsigned char readRegMPU(unsigned char reg);
  unsigned char findMPU();
  unsigned char checkMPU();
  void filterMPUData();
  processedMpu processMPUData();

  
  void processAngles(processedMpu dados);

  void setSleepOff();
  void setGyroScale();
  void setAccelScale();
  
  mpu   _rawData;
  processedMpu _processedData; 

  float calGyX = 0;
  float calGyY = 0;
  float calGyZ = 0;

  float calAcX = 0;
  float calAcY = 0;

  unsigned long _lastTimestamp = 0;

  angles _ang;
  processedAngles _procAng;
   
  unsigned char led_state = 0;
  unsigned char debbuging_enabled = 1;
};
 
#endif
