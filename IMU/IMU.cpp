/* ***************************************************************** */
/* File name:        IMU.cpp                                         */
/* File description: MPU-6050 interface implementation file          */
/* Author name:      Giacomo Dollevedo, Gustavo Fernandes            */
/* Creation date:    18nov2020                                       */
/* Revision date:    02jan2021                                       */
/* ***************************************************************** */

#include "IMU.h"


/* ************************************************************************************ */
/* Method's name:          writeRegMPU                                                  */ 
/* Description:            Writes to a MPU-6005 register through I2C bus                */
/*                                                                                      */
/* Entry parameters:       int reg -> Register to write to                              */
/*                         int val -> Value to write                                    */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void IMU::writeRegMPU(int reg, int val)      //aceita um registro e um valor como parâmetro
{
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                      // escreve o valor no registro
  Wire.endTransmission(true);           // termina a transmissão
}

/* ************************************************************************************ */
/* Method's name:          readRegMPU                                                   */ 
/* Description:            Reads from a MPU-6005 register through I2C bus               */
/*                                                                                      */
/* Entry parameters:       unsigned char reg -> Register to read from                   */
/*                                                                                      */
/* Return parameters:      unsigned char -> value that was read                         */
/* ************************************************************************************ */
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

/* ************************************************************************************ */
/* Method's name:          findMPU                                                      */ 
/* Description:            Check for MPU-6050 address on I2C bus                        */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      unsigned char -> 0 == not found / 1 == found                 */
/* ************************************************************************************ */
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


/* ************************************************************************************ */
/* Method's name:          checkMPU                                                     */ 
/* Description:            Check MPU-6050 status through I2C bus                        */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      unsigned char -> 0 = not available / 1 = Active / 2 = Sleep  */
/* ************************************************************************************ */
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

/* ************************************************************************************ */
/* Method's name:          initMPU                                                      */ 
/* Description:            Initialize I2C bus and MPU-6050                              */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void IMU::initMPU()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  setSleepOff();
  setGyroScale();
  setAccelScale();
  checkMPU();
}

/* ************************************************************************************ */
/* Method's name:          setSleepOff                                                  */ 
/* Description:            Writes to specific register on MPU-6050 to set Active Mode   */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void IMU::setSleepOff()
{
  writeRegMPU(PWR_MGMT_1, 0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE
}

/* ************************************************************************************ */
/* Method's name:          setGyroScale                                                 */ 
/* Description:            Set gyroscope scale to +- 250°/s                             */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void IMU::setGyroScale()
{
  writeRegMPU(GYRO_CONFIG, 0);
}

/* ************************************************************************************ */
/* Method's name:          setAccelScale                                                */ 
/* Description:            Set accelerometer scale to +- 2g                             */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void IMU::setAccelScale()
{
  writeRegMPU(ACCEL_CONFIG, 0);
}

/* ************************************************************************************ */
/* Method's name:          readRawMPU                                                   */ 
/* Description:            Reads all sensor registers from MPU-6050 through I2C bus     */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      mpu -> Struct containing raw read values                     */
/* ************************************************************************************ */
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
void IMU::CalibrateGyro(float X, float Y, float Z){
  calGyX = X;
  calGyY = Y;
  calGyZ = Z;
}

/* ************************************************************************************ */
/* Method's name:          CalibrateAcl                                                 */ 
/* Description:            Set accelerometer calibration values for baseline shift      */
/*                                                                                      */
/* Entry parameters:       float X -> X axis calibration value                          */
/*                         float Y -> Y axis calibration value                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void IMU::CalibrateAcl(float X, float Y){
  calAcX = X;
  calAcY = Y;
}


/* ************************************************************************************ */
/* Method's name:          getRawData                                                   */ 
/* Description:            Returns internal raw sensor data struct                      */
/*                                                                                      */
/* Entry parameters:                                                                    */
/*                                                                                      */
/* Return parameters:      mpu -> raw data struct                                       */
/* ************************************************************************************ */
mpu IMU::getRawData(){
  return _rawData;
}


/* ************************************************************************************ */
/* Method's name:          processMPUData                                               */ 
/* Description:            Converts raw data to actual values. Also finds angular       */
/*                         displacement                                                 */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      processedMpu -> processed data struct                        */
/* ************************************************************************************ */
processedMpu IMU::processMPUData(){

  /*Convertendo o dado lido do MPU-6050 de acordo com a
  sensibilidade do giroscopio*/
  _processedData.GyX = (float)(_rawData.GyX - calGyX)/131;
  _processedData.GyY = (float)(_rawData.GyY - calGyY)/131;
  _processedData.GyZ = (float)(_rawData.GyZ - calGyZ)/131;

  /*Convertendo o dado lido do MPU-6050 de acordo com a
  sensibilidade do acelerometro*/
  _processedData.AcX = (float)(_rawData.AcX - calAcX)/16384;
  _processedData.AcY = (float)(_rawData.AcY- calAcY)/16384;
  _processedData.AcZ = (float)_rawData.AcZ/16384;

  /*Utilizando as informacoes de velocidade angular e aceleracao
  para calcular a posicao angular*/
  processAngles(_processedData);

  /*Dado de temperatura da MPU-6050*/
  _processedData.Tmp = (float)_rawData.Tmp;

  

  return _processedData;
}


/* ************************************************************************************ */
/* Method's name:          processAngles                                                */ 
/* Description:            Converts gyro and accel data into angular displacement       */
/*                                                                                      */
/* Entry parameters:       processedMpu dados -> data struct to process                 */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void IMU::processAngles(processedMpu dados){

  unsigned long aux = micros();
  float x2, y2, z2, result;

  /*Calculo do tempo percorrido desde a ultima medida*/
  float dt = (float)(aux - _lastTimestamp)/1000000;

  /*Guardando o tempo desta medida*/
  _lastTimestamp = aux;

  //Serial.println(dt);

  /*Posicao angular medida somente pela integracao da velocidade*/
  _gyroPitch += _processedData.GyX*dt;  //Angulo de Pitch
  _gyroRoll  += _processedData.GyY*dt;  //Angulo de Roll

  /*Valores da ultima iteracao dos angulos*/
  _ang.GyPitch = _procAng.Pitch;
  _ang.GyRoll  = _procAng.Roll;
  _ang.GyYaw   = _procAng.Yaw;
  
  /*Posicao angular atraves da integracao da velocidade
  considerando a ultima iteracao*/
  _ang.GyPitch  += _processedData.GyX*dt; //MPU eixo X

  /*Compensacao de giro em Yaw na posicao angular*/
  if(yaw_compensation)
    _ang.GyPitch  += _ang.GyRoll*sin(_ang.GyYaw*dt*DEG_2_RAD);

   /*Posicao angular atraves da integracao da velocidade
  considerando a ultima iteracao*/
  _ang.GyRoll   += _processedData.GyY*dt; //MPU eixo Y

  /*Compensacao de giro em Yaw na posicao angular*/
  if(yaw_compensation)
    _ang.GyRoll   -= _ang.GyPitch*sin(_ang.GyYaw*dt*DEG_2_RAD);
  
  _ang.GyYaw    += _processedData.GyZ*dt; //MPU eixo Z
  
/*
  Serial.printf("Acelerometer INPUT:\t %f\t%f\t%f\n",_processedData.AcX, _processedData.AcY, _processedData.AcZ);
*/

  /*CALCULO DA POSICAO ANGULAR ATRAVES DO ACELEROMETRO*/

  /*Elevando os dados ao quadrado*/
  x2 = _processedData.AcX*_processedData.AcX;
  y2 = _processedData.AcY*_processedData.AcY;
  z2 = _processedData.AcZ*_processedData.AcZ;

  /*Decomposicao do vetor da gravidade*/
  //Pitch
  result = sqrt(x2+z2);
  /*Arco-Tangente para encontra o angulo apos decomposicao*/
  _ang.AclPitch = -1*atan2(-1*_processedData.AcY, result)*RAD_2_DEG;

  //Roll
  /*Arco-Tangente para encontra o angulo apos decomposicao*/
  _ang.AclRoll  = -1*atan2(_processedData.AcX, _processedData.AcZ)*RAD_2_DEG;

  /*Aplicacao do filtro complementar para obter o angulo final*/
  filterMPUData();

  if(debbuging_enabled)
    Serial.printf("%f,%f,%f\n",_gyroRoll, _ang.AclRoll, _procAng.Roll);

/*
  Serial.printf("ANGULOS GIROSCOPIO:\nROLL:%f\tPITCH:%f\t%YAW:%f\n",_ang.GyRoll, _ang.GyPitch, _ang.GyYaw);
  Serial.printf("ANGULOS ACELEROMETRO:\nROLL:%f\tPITCH:%f\n",_ang.AclRoll, _ang.AclPitch);
  Serial.printf("ANGULOS COMPLEMENTAR:\nROLL:%f\tPITCH:%f\t%YAW:%f\n\n", _procAng.Roll, _procAng.Pitch, _procAng.Yaw);
*/
}


/* ************************************************************************************ */
/* Method's name:          filterMPUData                                                */ 
/* Description:            Complementary filter to keep angular displacement from       */
/*                         drifting                                                     */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void IMU::filterMPUData( ){

  /*Atribuindo 'pesos' do filtro para as medidas de cada sensor
  e construindo o sinal final.*/
  _procAng.Roll   =   CF_GY*_ang.GyRoll + CF_AC*_ang.AclRoll;
  _procAng.Pitch  =   CF_GY*_ang.GyPitch + CF_AC*_ang.AclPitch;
  _procAng.Yaw    =   _ang.GyYaw;


}



/* ************************************************************************************ */
/* Method's name:          update                                                       */ 
/* Description:            Reads from MPU-6050 and process data, updating internal      */
/*                         values                                                       */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void IMU::update(){
  
  unsigned char aux = 0;
  float auxRollVel = 0;
  float auxPitchVel = 0;
  
  readRawMPU();
  processMPUData();


  /*if(_meanPos == 9){
    _meanPos = 0;
  }

  _roll_vel[_meanPos] = _processedData.GyY;
  _pitch_vel[_meanPos] = _processedData.GyX;

  while(aux != 10){
    auxRollVel  += _roll_vel[aux];
    auxPitchVel += _pitch_vel[aux];
    aux++;
  }

  _meanVel.Roll = auxRollVel/10;
  _meanVel.Pitch = auxPitchVel/10;

  _meanPos++;*/

  _gyroRollInput = (_gyroRollInput*0.8) + (_processedData.GyY*0.2);
  _gyroPitchInput = (_gyroPitchInput*0.8) + (_processedData.GyX*0.2);

}

/* ************************************************************************************ */
/* Method's name:          getPitchVel                                                  */ 
/* Description:            Returns gyro pitch velocity after complementary filter       */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      float -> Pitch velocity                                      */
/* ************************************************************************************ */
float IMU::getPitchVel(){
  return _gyroPitchInput;

}

/* ************************************************************************************ */
/* Method's name:          getRollVel                                                   */ 
/* Description:            Returns gyro roll velocity after complementary filter        */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      float -> Roll velocity                                       */
/* ************************************************************************************ */
float IMU::getRollVel(){
  return _gyroRollInput;

}

/* ************************************************************************************ */
/* Method's name:          getData                                                      */ 
/* Description:            Returns internal processed data struct                       */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      processedMpu -> internal processed data struct               */
/* ************************************************************************************ */
processedMpu IMU::getData(){

  return _processedData;

}


/* ************************************************************************************ */
/* Method's name:          getRawAngles                                                 */ 
/* Description:            Returns raw angles from gyro and accelerometer calculation   */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      angles -> internal processed data struct                     */
/* ************************************************************************************ */
angles IMU::getRawAngles(){
  return _ang;
}


/* ************************************************************************************ */
/* Method's name:          getRotations                                                 */ 
/* Description:            Returns processed angular displacement after the filter      */
/*                         on Roll, Pitch and Yaw                                       */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      _procAng -> internal processed angular displacement struct   */
/* ************************************************************************************ */
processedAngles IMU::getRotations(){
  return _procAng;
}

/* ************************************************************************************ */
/* Method's name:          getGyroVel                                                   */ 
/* Description:            Returns the velocity mean from the gyroscope sensor          */
/*                         on Roll, Pitch and Yaw                                       */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      gyroVel -> internal mean velocities struct                   */
/* ************************************************************************************ */ 
gyroVel IMU::getGyroVel(){
  return _meanVel;
}


/* ************************************************************************************ */
/* Method's name:          enableDebug                                                  */ 
/* Description:            Enables serial communication for debbugging                  */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void IMU::enableDebug(){
  debbuging_enabled = 1;
}


/* ************************************************************************************ */
/* Method's name:          disableDebug                                                 */ 
/* Description:            Disables serial communication for debbugging                 */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
void IMU::disableDebug(){
  debbuging_enabled = 0;
}

/* ************************************************************************************ */
/* Method's name:          disableYawComp                                               */ 
/* Description:            Disables roll and pitch angle compensation using yaw         */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
 void disableYawComp(){
   yaw_compensation = 0;
 }


/* ************************************************************************************ */
/* Method's name:          enableYawComp                                                */ 
/* Description:            Enables roll and pitch angle compensation using yaw          */
/*                                                                                      */
/* Entry parameters:       n/a                                                          */
/*                                                                                      */
/* Return parameters:      n/a                                                          */
/* ************************************************************************************ */
 void enableYawComp(){
   yaw_compensation = 1;
 }