/*******************************************************************
 * Descrição: Arquivo c++ que implementa o controlador de voo do drone.
 * Autores: Gustavo L. Fernandes e Giácomo A. Dollevedo
 * Ultima Atualização: 14/11/2020
********************************************************************/

#include "Arduino.h"
#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>

/* ******************************************************************************** */
/* Nome do metodo:          FlighControl  - Construtor Vazio                        */
/* Descrição:               Inicializa um objeto controlador padrão, ou seja        */
/*                          com valores de ganho nulos                              */
/*                          Setpoint nulo e constante                               */
/*                                                                                  */
/* Parametros de entrada:  Nenhum (Vazio)                                           */
/*                                                                                  */
/*                                                                                  */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */

void FlightControl::FlighControl(char Axis)
{

//Ganho de cada um dos controladores, proporcional, integrativo e derivativo
    _kPID.fkp = 0; 
    _kPID.fki = 0
    _kPID.fkd = 0; 
    _axis = Axis;


//Set point de entrada para o controlador (Estático) 
    setPoint = 0;
}

/* ******************************************************************************** */
/* Nome do metodo:          FlighControl  - Construtor Pre Definido                 */
/* Descrição:               Inicializa um objeto controlador com ganhos definidos   */
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: (float fkp, float fki, float fkd) ganhos dos controladores*/
/*                        proporcional, integrador e derivativo                     */
/*                         Setpoint nulo e constante                                */
/*                        (char Axis) podendo ser "r", "p", "y" para determinar     */
/*                        qual o eixo que o objeto será definido                    */
/*                                                                                  */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */

void FlightControl::FlighControl(float fkp, float fki, float fkd, char Axis)
{

//Ganho de cada um dos controladores, proporcional, integrativo e derivativo
    kPID.fkp = fkp; 
    kPID.fki = fki
    kPID.fkd = fkd; 
    _axis = Axis;

//Set point de entrada para o controlador (Estático) 
    setPoint = 0;
}


/* ******************************************************************************** */
/* Nome do metodo:          PID                                                     */
/* Descrição:                - Rotina de Controle para um determinado eixo          */
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: (IMU imu) Objeto que contem os valores lidos da IMU       */
/*                        que será dado como entrada pro controlador                */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */


void FlightControl::pidControl(IMU imu)
{
//YAW = Z
//ROLL = Y
//PITCH = X
//Taxas de variação angular em cada um dos eixos da IMU
    float fgyro; 

//Testa qual eixo o objeto foi relacionado para coletar a informação adequada da IMU
    switch (_axis)
    {
    case "r":
        fgyro = imu.getData().Gyy;
        break;
    case "p":
        fgyro = imu.getData().Gyx;
        break;
    case "y"
        fgyro = imu.getData().Gyz;
    default:
        break;
    }

    
//Erro instantaneo e acumulado 
    float ferror_temp = fgyroRoll - _setpoint; 
    _fimem += _gains.fki * frollError_temp; 

//Sinal de saida do PID e atualização do erro previo

    _fpidCalculated = _gains.fkp*ferror_temp + _fimem + _gains.fkp*(ferror_temp - _ferror_previous); 
    _ferror_previous = ferror_temp;
}

/* ******************************************************************************** */
/* Nome do metodo:          getPID_Calculated                                       */
/* Descrição:                - retorna o valor de PID calculado a ser utilizado     */
/*                            para saida do sistema                                 */
/*                                                                                  */
/* Parametros de entrada: Nenhum (Vazio)                                            */
/*                                                                                  */
/* Parametros de saida: float _pidCalculated                                        */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */


float FlightControl::getPID_Calculated(){
    return _fpidCalculated;
}

/* ******************************************************************************** */
/* Nome do metodo:          getGains                                                */
/* Descrição:                - retorna o valor dos ganhos do controlador PID        */
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: Nenhum (Vazio)                                            */
/*                                                                                  */
/* Parametros de saida: kPID _gains                                                 */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */

kPID FlightControl::getGains(){
    return _gains;
}

/* ******************************************************************************** */
/* Nome do metodo:          setKp                                                   */
/* Descrição:                - Seta um novo valor para o ganho do controlador       */
/*                             proporcional                                         */
/*                                                                                  */
/* Parametros de entrada: float kp                                                  */
/*                                                                                  */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */

void FlightControl::setKp(float kp){
    _gains.fkp = kp;
}

/* ******************************************************************************** */
/* Nome do metodo:          setKp                                                   */
/* Descrição:                - Seta um novo valor para o ganho do controlador       */
/*                             derivativo                                           */
/*                                                                                  */
/* Parametros de entrada: float kd                                                  */
/*                                                                                  */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */
void FlightControl::setKd(float kd){
    _gains.fkd = kd;

}

/* ******************************************************************************** */
/* Nome do metodo:          setKp                                                   */
/* Descrição:                - Seta um novo valor para o ganho do controlador       */
/*                             integrativo                                          */
/*                                                                                  */
/* Parametros de entrada: float kd                                                  */
/*                                                                                  */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */
void FlightControl::setKi(float ki){
    _gains.fki = ki;

}