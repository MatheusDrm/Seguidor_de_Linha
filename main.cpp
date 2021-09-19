// Habilite o server antes na simulação V-REP com o comando lua:
// simExtRemoteApiStart(portNumber) -- inicia servidor remoteAPI do V-REP

extern "C"
{
#include "remoteApi/extApi.h"
}

#include <iostream>
#include <string.h>

using namespace std;

// Variaveis para conexao do servidor
string serverIP = "127.0.0.1";
int serverPort = 19999;
int clientID = simxStart((simxChar *)serverIP.c_str(), serverPort, true, true, 2000, 5);

// Variaveis da inicialização dos motores

int leftMotorHandle = 0;
float vLeft = 0;
int rightMotorHandle = 0;
float vRight = 0;
float vMotor = 33;

// Variaveis PID 

float integral = 0;
float derivativa = 0;
float kp = 2;
float ki = 0.3;
float kd = 1;
float erro = 0;
float erro_anterior = 0;

float termoP, termoI, termoD, saida; 


// Variaveis aux
int parar = 0;
int cruzamento = 0;
int contadorVolta = 0;
int aux = 1;
int auxTempo = 1;

void func_P(void);

void curva(void);

void tempo(void);

int main(int argc, char **argv)
{
  //Handles e nomes dos sensores
  string sensorNome[6] = {"sensor0", "sensor1", "sensor2", "sensor3", "sensor4", "camera"};
  int sensorHandle[6];

  int res[2];
  simxUChar* image;
  int sensorResponse[6] = {0,0,0,0,0,0};

  simxFloat* auxValues;

  simxUChar* detectionState;

  simxInt* auxValuesCount;
  //Tenta estabelecer conexao com a simulacao (nao esqueca de dar play)
  //int clientID = simxStart((simxChar *)serverIP.c_str(), serverPort, true, true, 2000, 5);

  //Se a conexao e estabelicida, sera retornado um valo diferente de 0
  if (clientID != -1)
  {
    printf("Servidor conectado!\n");
    
    // inicialização dos motores
    if (simxGetObjectHandle(clientID, (const simxChar *)"Roda_1", (simxInt *)&leftMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
      printf("Handle do motor esquerdo nao encontrado!\n");
    else
      printf("Conectado ao motor esquerdo!\n");

    if (simxGetObjectHandle(clientID, (const simxChar *)"Roda_2", (simxInt *)&rightMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
      printf("Handle do motor direito nao encontrado!\n");
    else
      printf("Conectado ao motor direito!\n");

    
    // inicialização dos sensores (remoteApi)
    for (int i = 0; i < 6; i++)
    {
      if (simxGetObjectHandle(clientID, (const simxChar *)sensorNome[i].c_str(), (simxInt *)&sensorHandle[i], (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
      {
        printf("Handle do sensor %s nao encontrado!\n",sensorNome[i].c_str());
      }
      else
      {
        printf("Conectado ao sensor %s \n",sensorNome[i].c_str());
        simxGetVisionSensorImage(clientID,sensorHandle[i],res,&image,0,simx_opmode_streaming);
      }
    }
    // inicialização da camera
    for (int i = 0; i < 5; i++)
    {
      if (simxGetObjectHandle(clientID, (const simxChar *)sensorNome[i].c_str(), (simxInt *)&sensorHandle[i], (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
      {
        printf("Handle do sensor %s nao encontrado!\n",sensorNome[i].c_str());
      }
      else
      {
        printf("Conectado ao sensor %s \n",sensorNome[i].c_str());
        simxGetVisionSensorImage(clientID,sensorHandle[i],res,&image,0,simx_opmode_streaming);
      }
    }


    //                                                      CÓDIGO BASE


    while (simxGetConnectionId(clientID) != -1) // enquanto a simulação estiver ativa
    {
      for (int i = 0; i < 5; i++)
      {
        if(simxGetVisionSensorImage(clientID,sensorHandle[i],res,&image,0,simx_opmode_streaming) == simx_return_ok)
        {
          int sum = 0;
          
          for (int i = 0; i < res[0]*res[0]; i++)
          {
            sum += (int) image[i];
          }
          sum = sum/(res[0]*res[0]);
          if(sum > 120)
            sensorResponse[i] = 1; //branco
          else
            sensorResponse[i] = 0; //preto
        } 
      }
     if(sensorResponse[2] == 0){
       cruzamento = 0;
       parar = 0;
     }

      //                                                 IMPLEMENTAÇÃO DO PID

      // Calculo do erro
      if(parar==0){
        if (sensorResponse[0]>0.5 && sensorResponse[1]>0.5 && sensorResponse[2]>0.5 && sensorResponse[3]>0.5 && sensorResponse[4]<0.5){
                erro = 4;
           
        }else if (sensorResponse[0]>0.5 && sensorResponse[1]>0.5 && sensorResponse[2]>0.5 && sensorResponse[3]<0.5 && sensorResponse[4]<0.5) 
                erro = 3;
               
         else if (sensorResponse[0]>0.5 && sensorResponse[1]>0.5 && sensorResponse[2]>0.5 && sensorResponse[3]<0.5 && sensorResponse[4]>0.5) 
                erro = 2;
             
         else if (sensorResponse[0]>0.5 && sensorResponse[1]>0.5 && sensorResponse[2]<0.5 && sensorResponse[3]<0.5 && sensorResponse[4]>0.5) 
                erro = 1;
             
         else if (sensorResponse[0]>0.5 && sensorResponse[1]>0.5 && sensorResponse[2]<0.5 && sensorResponse[3]>0.5 && sensorResponse[4]>0.5) 
                erro = 0;
              
         else if (sensorResponse[0]>0.5 && sensorResponse[1]<0.5 && sensorResponse[2] <0.5 && sensorResponse[3]>0.5 && sensorResponse[4]>0.5) 
                erro = -1;
              
         else if (sensorResponse[0]>0.5 && sensorResponse[1]<0.5 && sensorResponse[2]>0.5 && sensorResponse[3]>0.5 && sensorResponse[4]>0.5)
                erro = -2;
              
         else if (sensorResponse[0]<0.5 && sensorResponse[1]<0.5 && sensorResponse[2]>0.5 && sensorResponse[3]>0.5 && sensorResponse[4]>0.5)
                erro = -3;
            
         else if (sensorResponse[0]<0.5 && sensorResponse[1]>0.5 && sensorResponse[2]>0.5 && sensorResponse[3]>.5 && sensorResponse[4]>0.5)
                erro = -4;
            
         else if (sensorResponse[0]>0.5 && sensorResponse[1]>0.5 && sensorResponse[2]>0.5 && sensorResponse[3]>.5 && sensorResponse[4]>0.5){
                if (erro_anterior <= -4) {
                    erro = -5; 
                }else
                    erro = 5;
          }        
        func_P();
        printf("%f\n", saida);
       }

      //                                                  CÓDIGO DA VISÃO
      
      if(simxGetVisionSensorImage(clientID,sensorHandle[5],res,&image,1,simx_opmode_streaming) == simx_return_ok){
        //printf("Camera reconhecida");
      }else{
        //printf("Camera nao reconhecida");
      }
      
      if(simxReadVisionSensor(clientID,sensorHandle[5], detectionState, &auxValues, &auxValuesCount,simx_opmode_streaming) == simx_return_ok){
          printf("Profundidade: %f\n", auxValues[14]);
          //                                CÓDIGO BLOCO AZUL
          /*if (auxValues[14]<0.04 && cruzamento==0){
            simxSetJointTargetVelocity(clientID, leftMotorHandle, 5, simx_opmode_streaming);
            simxSetJointTargetVelocity(clientID, rightMotorHandle, 5, simx_opmode_streaming);
            if (auxValues[14]<0.026){
              cruzamento = 1;
              simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_streaming);
              simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_streaming);
              curva();
              
            }
            parar = 1;
            saida = 0;
          }*/
          //                               CÓDIGO BLOCO VERMELHO
          if (auxValues[14]<0.026){
              if (auxTempo==1){
                auxTempo=0;
                tempo();
              }
            //parar = 1;
            //saida = 0;
            }
      }else{
        printf("DEU RUIM");
      }
    
      extApi_sleepMs(50);
    }
    simxFinish(clientID); // fechando conexao com o servidor
    cout << "Conexao fechada!" << std::endl;
  }
  else

  {
    printf("Problemas para conectar o servidor!\n");
  }
  return 0;
}

// FUNÇÃO PARA IMPLEMENTAÇÃO DO CONTROLE PID

void func_P(void){
        termoP = erro*kp;

        integral += erro;
        termoI = ki*integral;

        derivativa = erro-erro_anterior;
        termoD = derivativa*kd;

        saida = termoP + termoD + termoI;
        erro_anterior = erro; 

        vLeft = (vMotor + saida) * 0.4;
        vRight = (vMotor - saida) * 0.4;

        
        simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat)vLeft, simx_opmode_streaming);
        simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat)vRight, simx_opmode_streaming);
}

// FUNÇÃO PARA REALIZAÇÃO DAS CURVAS

void curva(void){
  if (contadorVolta==0){
      simxSetJointTargetVelocity(clientID, rightMotorHandle, 5, simx_opmode_streaming);
  }else if (contadorVolta==1){
      simxSetJointTargetVelocity(clientID, leftMotorHandle, 5, simx_opmode_streaming);
  }
}

// FUNÇÃO PARA PARAR O TEMPO

void tempo(void){
  simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_streaming);
  simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_streaming);
  extApi_sleepMs(8000);
  //parar = 0;
  //cruzamento = 0;
}