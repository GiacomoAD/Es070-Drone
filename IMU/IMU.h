/* ***************************************************************** */
/* File name:        IMU.h                                           */
/* File description: MPU-6050 interface header file                  */
/* Author name:      Giacomo Dollevedo, Gustavo Fernandes            */
/* Creation date:    18nov2020                                       */
/* Revision date:    15dec2020                                       */
/* ***************************************************************** */

 
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

#define CF_GY          0.99 //  Fator Gyro filtro complementar
#define CF_AC          0.01 //  Fator Accel filtro complementar

#define RAD_2_DEG      57.2958  //  Conversao Radianos para Graus
#define DEG_2_RAD      0.01745  //  Conversao Graus para Radianos

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

typedef struct gyroVel
{
  float Roll = 0;
  float Pitch = 0;
  float Yaw = 0;
};

class IMU
{
public:
/* ************************************************************************************ */
/* Method's name:          initMPU                                                      */ 
/* Description:            Initialize I2C bus and MPU-6050                              */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
 void initMPU();


/* ************************************************************************************ */
/* Method's name:          readRawMPU                                                   */ 
/* Description:            Reads all sensor registers from MPU-6050 through I2C bus     */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      mpu -> Struct containing raw read values                     */
/* ************************************************************************************ */ 
 mpu readRawMPU();

/* ************************************************************************************ */
/* Method's name:          getData                                                      */ 
/* Description:            Returns internal processed data struct                       */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      processedMpu -> internal processed data struct               */
/* ************************************************************************************ */ 
 processedMpu getData();


/* ************************************************************************************ */
/* Method's name:          getRawData                                                   */ 
/* Description:            Returns internal raw sensor data struct                      */
/*                                                                                      */
/* Entry parameters:                                                                    */
/*                                                                                      */
/* Return parameters:      mpu -> raw data struct                                       */
/* ************************************************************************************ */
 mpu getRawData();


/* ************************************************************************************ */
/* Method's name:          getRawAngles                                                 */ 
/* Description:            Returns raw angles from gyro and accelerometer calculation   */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      angles -> internal processed data struct                     */
/* ************************************************************************************ */ 
 angles getRawAngles();


/* ************************************************************************************ */
/* Method's name:          getRotations                                                 */ 
/* Description:            Returns processed angular displacement after the filter      */
/*                         on Roll, Pitch and Yaw                                       */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      _procAng -> internal processed angular displacement struct   */
/* ************************************************************************************ */ 
 processedAngles getRotations();


/* ************************************************************************************ */
/* Method's name:          getGyroVel                                                   */ 
/* Description:            Returns the velocity mean from the gyroscope sensor          */
/*                         on Roll, Pitch and Yaw                                       */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      gyroVel -> internal mean velocities struct                   */
/* ************************************************************************************ */ 
gyroVel getGyroVel();


float getPitchVel();

float getRollVel();

/* ************************************************************************************ */
/* Method's name:          CalibrateGyro                                                */ 
/* Description:            Set gyro calibration values for baseline shift               */
/*                                                                                      */
/* Entry parameters:       float X -> X axis calibration value                          */
/*                         float Y -> Y axis calibration value                          */
/*                         float Z -> Z axis calibration value                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
 void CalibrateGyro(float X, float Y, float Z);
 

/* ************************************************************************************ */
/* Method's name:          CalibrateAcl                                                 */ 
/* Description:            Set accelerometer calibration values for baseline shift      */
/*                                                                                      */
/* Entry parameters:       float X -> X axis calibration value                          */
/*                         float Y -> Y axis calibration value                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */ 
 void CalibrateAcl(float X, float Y);
 
/* ************************************************************************************ */
/* Method's name:          update                                                       */ 
/* Description:            Reads from MPU-6050 and process data, updating internal      */
/*                         values                                                       */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */ 
 void update();


/* ************************************************************************************ */
/* Method's name:          enableDebug                                                  */ 
/* Description:            Enables serial communication for debbugging                  */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
 void enableDebug();


/* ************************************************************************************ */
/* Method's name:          disableDebug                                                 */ 
/* Description:            Disables serial communication for debbugging                 */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
 void disableDebug();

 
private:

/* ************************************************************************************ */
/* Method's name:          writeRegMPU                                                  */ 
/* Description:            Writes to a MPU-6005 register through I2C bus                */
/*                                                                                      */
/* Entry parameters:       int reg -> Register to write to                              */
/*                         int val -> Value to write                                    */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
  void writeRegMPU(int reg, int val);


/* ************************************************************************************ */
/* Method's name:          readRegMPU                                                   */ 
/* Description:            Reads from a MPU-6005 register through I2C bus               */
/*                                                                                      */
/* Entry parameters:       unsigned char reg -> Register to read from                   */
/*                                                                                      */
/* Return parameters:      unsigned char -> value that was read                         */
/* ************************************************************************************ */
  unsigned char readRegMPU(unsigned char reg);


/* ************************************************************************************ */
/* Method's name:          findMPU                                                      */ 
/* Description:            Check for MPU-6050 address on I2C bus                        */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      unsigned char -> 0 == not found / 1 == found                 */
/* ************************************************************************************ */
  unsigned char findMPU();


/* ************************************************************************************ */
/* Method's name:          checkMPU                                                     */ 
/* Description:            Check MPU-6050 status through I2C bus                        */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      unsigned char -> 0 = not available / 1 = Active / 2 = Sleep  */
/* ************************************************************************************ */
  unsigned char checkMPU();


 /* ************************************************************************************ */
/* Method's name:          filterMPUData                                                */ 
/* Description:            Complementary filter to keep angular displacement from       */
/*                         drifting                                                     */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */ 
  void filterMPUData();


/* ************************************************************************************ */
/* Method's name:          processMPUData                                               */ 
/* Description:            Converts raw data to actual values. Also finds angular       */
/*                         displacement                                                 */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      processedMpu -> processed data struct                        */
/* ************************************************************************************ */
  processedMpu processMPUData();


/* ************************************************************************************ */
/* Method's name:          processAngles                                                */ 
/* Description:            Converts gyro and accel data into angular displacement       */
/*                                                                                      */
/* Entry parameters:       processedMpu dados -> data struct to process                 */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */ 
  void processAngles(processedMpu dados);


/* ************************************************************************************ */
/* Method's name:          setSleepOff                                                  */ 
/* Description:            Writes to specific register on MPU-6050 to set Active Mode   */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
  void setSleepOff();


/* ************************************************************************************ */
/* Method's name:          setGyroScale                                                 */ 
/* Description:            Set gyroscope scale to +- 250°/s                             */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
  void setGyroScale();


/* ************************************************************************************ */
/* Method's name:          setAccelScale                                                */ 
/* Description:            Set accelerometer scale to +- 2g                             */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
  void setAccelScale();


  mpu   _rawData;
  processedMpu _processedData; 

  float calGyX = 0;
  float calGyY = 0;
  float calGyZ = 0;

  float calAcX = 0;
  float calAcY = 0;

  float _gyroRoll = 0;
  float _gyroPitch = 0;
  float _gyroRollInput = 0;
  float _gyroPitchInput = 0;
  

  unsigned long _lastTimestamp = 0;

  angles _ang;
  processedAngles _procAng;
   
  unsigned char led_state = 0;
  unsigned char debbuging_enabled = 1;

  unsigned char _meanPos = 0;
  float _roll_vel[50] = {0};
  float _pitch_vel[50] = {0};

  gyroVel _meanVel;

};
 
#endif
