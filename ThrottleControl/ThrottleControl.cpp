/*******************************************************************
 * Descrição: Arquivo c++ que implementa a interface de controle 
 * de velocidade dos motores do Drone
 * Autores: Gustavo L. Fernandes e Giácomo A. Dollevedo
 * Ultima Atualização: 18/11/2020
********************************************************************/
#include "ThrottleControl.h"


/* ******************************************************************************** */
/* Nome do metodo:          ThrottleControl                                         */
/* Descrição:               Método construtor do objeto, não faz nada além de       */
/*                          associar 4 objetos do tipo Servo que serão utilizados   */
/*                                                                                  */
/* Parametros de entrada:    Nenhum (Vazio)                                         */
/*                                                                                  */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */
ThrottleControl::ThrottleControl() 
{ 
  Serial.begin(9600);
  Serial.println("Objeto de Drone Criado!");
}

/* ******************************************************************************** */
/* Nome do metodo:          initializeMotors                                        */
/* Descrição:              Inicia cada um dos motores criados, atrelando a uma saída*/
/*                         PWM do ESP32 e setando os limites max e min que define   */
/*                         também a inicialização dos 4 ESC                         */
/*                                                                                  */
/* Parametros de entrada: int pinMotor1, int pinMotor2, int pinMotor3, int pinMotor4*/
/*                        Pinos PWM que serão conectados os sinais de controle      */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */

void ThrottleControl::initializeMotors(int pinMotor1, int pinMotor2, int pinMotor3, int pinMotor4)
{

    _m1.attach(pinMotor1,1000, 2000); 
    _m2.attach(pinMotor2,1000, 2000);
    _m3.attach(pinMotor3,1000, 2000);
    _m4.attach(pinMotor4,1000, 2000);
    
    _m1.write(1000);
    _m2.write(1000);

    _m3.write(1000);
    _m4.write(1000);

     delay(2000); 


}

/* ******************************************************************************** */
/* Nome do metodo:         setThrottle                                               */
/* Descrição:              Define as velocidades que serão atingidas pelos 4 motores*/
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: int desiredVel1, int desiredVel2, int desiredVel3,        */
/*                        int desiredVel4 , que são as velocidades desejadas        */
/*                        aceita valores entre 1000 e 2000 mas limita em 1500       */
/* Parametros de saida: Nenhum (Vazio)                                              */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */

void ThrottleControl::setThrottle(int desiredVel1, int desiredVel2, int desiredVel3, int desiredVel4)
{
//  if((desiredVel1 < 1500) & (desiredVel2 < 1500) & (desiredVel3 < 1500) & (desiredVel4 < 1500)){
    _m1.write(desiredVel1); 
    _m2.write(desiredVel2); 
    _m3.write(desiredVel3);
    _m4.write(desiredVel4);
//  }
}

/* ******************************************************************************** */
/* Nome do metodo:         getTrottle                                               */
/* Descrição:              Consulta a ultima velocidade definida nos 4 motores      */
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: Nenhum (Vazio)                                            */
/*                                                                                  */
/* Parametros de saida:   int[3] array com os 4 valores de velocidades lidos        */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */


int* ThrottleControl::getThrottle(){ 
    int* ActualVel = (int*) calloc(3, sizeof(int));

    ActualVel[0] = _m1.read();
    ActualVel[1] = _m2.read();
    ActualVel[2] = _m3.read();
    ActualVel[3] = _m4.read();

    return ActualVel;
}

/* ******************************************************************************** */
/* Nome do metodo:         gtestMotors                                              */
/* Descrição:              Testa se os motores estão funcinoando e respondendo      */
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: Nenhum (Vazio)                                            */
/*                                                                                  */
/* Parametros de saida:   boolean  (1 Funcionando) (0 Com problemas)                */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */

boolean ThrottleControl:: testMotors(){


}

/* ******************************************************************************** */
/* Nome do metodo:         ThrottleControl                                          */
/* Descrição:              Distribuia a velocidade controlada para os 4 motores     */
/*                                                                                  */
/*                                                                                  */
/* Parametros de entrada: FlightControl pidRoll, FlightControl pidPitch,            */
/*                        FlightControl pidYaw  , que são os objetos                */
/*                        do controlador implementado para cada eixo                */
/*                                                                                  */
/* Parametros de saida:  Vazio (Nenhum)                                             */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */

void ThrottleControl::Control(FlightControl pidRoll, FlightControl pidPitch, FlightControl pidYaw ){ 

    int vel1, vel2, vel3, vel4;
    int desiredVel1, desiredVel2, desiredVel3, desiredVel4;
    int* actualVel = (int*) calloc(3, sizeof(int));


    //Pega a velocidade atual em cada um dos motores
    actualVel = getThrottle();
    vel1 = actualVel[0];
    vel2 = actualVel[1];
    vel3 = actualVel[2];
    vel4 = actualVel[3];


    //Calcula as compensações em cada motor para manter o controle de cada um dos eixos de movimentação
    desiredVel1 = vel1 - pidPitch.getPID_Calculated() + pidRoll.getPID_Calculated() - pidYaw.getPID_Calculated();
    desiredVel2 = vel2 + pidPitch.getPID_Calculated() + pidRoll.getPID_Calculated() + pidYaw.getPID_Calculated();
    desiredVel3 = vel3 + pidPitch.getPID_Calculated() - pidRoll.getPID_Calculated() - pidYaw.getPID_Calculated();
    desiredVel4 = vel4 - pidPitch.getPID_Calculated() - pidRoll.getPID_Calculated() + pidYaw.getPID_Calculated();

    //Vamos saturar as velocidades maximas em cada motor
    if(desiredVel1 > MAXTHROTTLE){
      desiredVel1 = MAXTHROTTLE;
    }
    if(desiredVel2 > MAXTHROTTLE){
      desiredVel2 = MAXTHROTTLE;
    }
    if(desiredVel3 > MAXTHROTTLE){
      desiredVel3 = MAXTHROTTLE;
    }
    if(desiredVel4 > 1500){
      desiredVel4 = 1500;
    }

    

    //Seta a nova velocidade necessária para manter a saida controlada
    setThrottle(desiredVel1, desiredVel2, desiredVel3, desiredVel4);

}

/* ******************************************************************************** */
/* Nome do metodo:         ThrottleControl                                          */
/* Descrição:              Distribui  a velocidade controlada para os 2 motores     */
/*                         de modo a contrlar apenas um eixo de movimento           */
/*                                                                                  */
/* Parametros de entrada: FlightControl pidRoll que é objeto do eixo roll de        */
/*                        controle            ,                                     */
/*                                                                                  */
/*                                                                                  */
/* Parametros de saida:  Vazio (Nenhum)                                             */
/*                                                                                  */
/*                                                                                  */
/* ******************************************************************************** */

void ThrottleControl::SingleAxisControl(FlightControl pidRoll){ 
    int vel1, vel2;
    int desiredVel1, desiredVel2;
    int* actualVel = (int*) calloc(3, sizeof(int));

   //Pega a velocidade atual em cada um dos motores
    actualVel = getThrottle();
    vel1 = actualVel[0];
    vel2 = actualVel[1];

     //Calcula as compensações em cada motor para manter o controle de cada um dos eixos de movimentação
    desiredVel1 = vel1  + pidRoll.getPID_Calculated() ;
    desiredVel2 = vel2  - pidRoll.getPID_Calculated() ;

    //Vamos saturar as velocidades maximas em cada motor
    if(desiredVel1 > MAXTHROTTLE){
      desiredVel1 = MAXTHROTTLE;
    }
    if(desiredVel2 > MAXTHROTTLE){
      desiredVel2 = MAXTHROTTLE;
    }

    //Seta a nova velocidade necessária para manter a saida controlada
    setThrottle(desiredVel1, desiredVel2 ,actualVel[2], actualVel[3] );
}
