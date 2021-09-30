// Habilite o server antes na simulação V-REP com o comando lua:
// simExtRemoteApiStart(portNumber) -- inicia servidor remoteAPI do V-REP

extern "C"
{
#include "remoteApi/extApi.h"
}

// Implementação das bibliotecas
#include <iostream>
#include <string.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

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

// Declaração das variaveis específicas do PID 

float integrativo = 0;
float derivativa = 0;
float kp = 3.3;
float ki = 0.4;
float kd = 1;
float erro = 0;
float erro_anterior = 0;

float termoP, termoI, termoD, saida; 


// Declaração das variaveis auxiliares

int parar = 0;
int cruzamento = 0;
int contadorVolta = 0;
int auxContador = 0;
int aux = 1;
int auxTempo = 1;

// Declaração das funções necessárias

void func_P(void);

void curva(void);

void tempo(void);

int detecAzul(const Mat& src);

int detecVermelho(const Mat& src);

int main(int argc, char **argv)
{
  //Handles e nomes dos sensores

  string sensorNome[6] = {"sensor0", "sensor1", "sensor2", "sensor3", "sensor4", "camera"};
  int sensorHandle[6];

  int res[2];
  simxUChar* image;

  int resC[2];
  simxUChar* camera;

  int sensorResponse[6] = {0,0,0,0,0,0};

  simxFloat* auxValues;

  simxUChar* detectionState;

  simxInt* auxValuesCount;

  // Teste da condição para conexão do Remote Api com o servidor 
  if (clientID != -1)
  {
    printf("Servidor conectado!\n");
    
    //                     Definição de variáveis Locais

    // Definição dos ranges de Hue, saturação e Value para as cores vermelho e azul
    Scalar blueLow = Scalar(100, 150, 150);
    Scalar blueHigh = Scalar(140, 255, 255);

    Scalar redLow = Scalar(0, 70, 50);
    Scalar redHigh = Scalar(10, 255, 255);

    /*  Variáveis para regulação do parâmetros do HSV da detecção de cores
    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0; 
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;
    */

    // Variável para estado de detecção das cores

    int azul = 0;
    int vermelho = 0;
    
    // Comandos para criar duas janelas, uma para mostrar o filtro da cor azul da imagem da camera e outra para mostrar o filtro da cor vermelho
    namedWindow("JanelaB", WINDOW_AUTOSIZE);
    namedWindow("JanelaR", WINDOW_AUTOSIZE);

    // Parte do código comentada, porém pode ser utilizada caso tenha interesse em fazer os ajustes dos parâmetros de forma mais fácil
    
    /*
    createTrackbar("LowH", "JanelaR", &iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "JanelaR", &iHighH, 179);

    createTrackbar("LowS", "JanelaR", &iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "JanelaR", &iHighS, 255);

    createTrackbar("LowV", "JanelaR", &iLowV, 255);//Value (0 - 255)
    createTrackbar("HighV", "JanelaR", &iHighV, 255);
    */

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


    //                                                     CÓDIGO BASE


    while (simxGetConnectionId(clientID) != -1) // Looping enquanto a simulação estiver ativa
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

      // Calculo do erro (erro é definido de acordo com a posição do seguidor em relação a linha preta)
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
       }

      cout << " contadorVolta: "<< contadorVolta;
      //                                                  CÓDIGO DA VISÃO
      
      // Receber imagem da camera
      if(simxGetVisionSensorImage(clientID,sensorHandle[5],res,&image,1,simx_opmode_streaming) == simx_return_ok){
        //printf("Camera reconhecida");
      }else{
        //printf("Camera nao reconhecida");
      }
      
      // Código base do processamento de imagem

      if(simxReadVisionSensor(clientID,sensorHandle[5], detectionState, &auxValues, &auxValuesCount,simx_opmode_streaming) == simx_return_ok){
        // Essa função retorna um vetor auxValues em que o 14 termo é a profundidade medida pela câmera e, portanto, será utilizada para medir a distância à objetos

          //                                CÓDIGO PARA DESAFIO DO BLOCO AZUL

          if(simxGetVisionSensorImage(clientID,sensorHandle[5],resC,&camera, 0,simx_opmode_streaming) == simx_return_ok)
              {
                // Definição da matriz para receber as imagens da camera
                Mat videoSensor( res[0], res[1], CV_8UC3, camera);
                // A imagem vem direcionada errada, logo é necessário inverter a imagem
                flip(videoSensor, videoSensor, 0);
                // Converter a imagem recebida que estava em RGB para BGR
                cvtColor(videoSensor, videoSensor, COLOR_RGB2BGR);
                Mat imgThresholdedB;
                Mat imgThresholdedR;
                // Converter a imagem de BGR para HSV para facilitar detecção de cor
                cvtColor(videoSensor, videoSensor, COLOR_BGR2HSV);
                // Limitar a imagem aos ranges de cor definidos anteriormente
                inRange(videoSensor, blueLow, blueHigh, imgThresholdedB);
                inRange(videoSensor, redLow, redHigh, imgThresholdedR);

                //  Processamento da imagem
                erode(imgThresholdedB, imgThresholdedB, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
                dilate( imgThresholdedB, imgThresholdedB, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

                erode(imgThresholdedR, imgThresholdedR, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
                dilate( imgThresholdedR, imgThresholdedR, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
              
                if(cruzamento==0){
                  azul = detecAzul(imgThresholdedB);
                  vermelho = detecVermelho(imgThresholdedR);
                }
                
                // Mostrar a imagem pós filtro de cor na janela criada anteriormente
                imshow("JanelaB",imgThresholdedB);
                imshow("JanelaR",imgThresholdedR);
                waitKey(10);
              }
          // Código que será executado caso o seguidor esteja a uma pequena distância do bloco azul
          if (auxValues[14]<0.045 && cruzamento==0 && azul==1){
            // Script para diminuir a velocidade enquanto o seguidor não chegar na distância adequada para realização da curva
            simxSetJointTargetVelocity(clientID, leftMotorHandle, 5, simx_opmode_streaming);
            simxSetJointTargetVelocity(clientID, rightMotorHandle, 5, simx_opmode_streaming);
            if (auxValues[14]<0.035){
              cruzamento = 1;
              simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_streaming);
              simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_streaming);
              // Chama função para realização da curva
              curva();       
            }
            parar = 1;
            saida = 0;
          }
          
          //                              CÓDIGO PARA DESAFIO DO BLOCO VERMELHO
          
         if (auxValues[14]<0.2 && vermelho==1){
              if (auxTempo==1){
                auxTempo=0;
                tempo();
              }
            }

            //                               CÓDIGO DE PARADA

          if (auxValues[14]<0.01 && azul==1 && contadorVolta >= 3){
            // Define as velocidades dos motores como 0 e define parar = 1 para evitar funcionamento do PID
            parar = 1;
            simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_streaming);
            simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_streaming);
            }
            
      }else{
        printf("Problemas para pegar imagem");
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
      // Aplicação da lógica do controle PID
        termoP = erro*kp;

        integrativo += erro;
        termoI = ki*integrativo;

        derivativa = erro-erro_anterior;
        termoD = derivativa*kd;

        saida = termoP + termoD + termoI;
        erro_anterior = erro; 

        vLeft = (vMotor + saida) * 0.4;
        vRight = (vMotor - saida) * 0.4;

        // Definindo as velocidades nos motores
        simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat)vLeft, simx_opmode_streaming);
        simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat)vRight, simx_opmode_streaming);
}

// FUNÇÃO PARA REALIZAÇÃO DAS CURVAS

void curva(void){
  if (contadorVolta==1){
    // Curva para a esquerda
      simxSetJointTargetVelocity(clientID, rightMotorHandle, 5, simx_opmode_streaming);
  }else if (contadorVolta==2){
      // Curva para a direita
      simxSetJointTargetVelocity(clientID, leftMotorHandle, 5, simx_opmode_streaming);
      contadorVolta = contadorVolta +1;
  }
}

// FUNÇÃO PARA PARAR O SEGUIDOR POR DETERMINADO TEMPO

void tempo(void){
  simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_streaming);
  simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_streaming);
  // Para o funcionamento do código por 5 segundos
  extApi_sleepMs(8000);
  auxContador = 0;
  parar = 0;
  cruzamento = 0;
}

//FUNÇÃO PARA DETECÇÃO DA COR AZUL

int detecAzul(const Mat& src){
  Moments MomentosB = moments(src);
  // Cálculo da área da imagem com cor azul a partir do momento
  double dAreaB = MomentosB.m00;
  // Condição para determinar leitura do azul
  if (dAreaB>5000000){
    if (auxContador==0){
      contadorVolta = contadorVolta + 1;
      auxContador = 1;
    }
    cruzamento = 0;
    return 1;
  }
  waitKey(10);
  return 0;
  }

//FUNÇÃO PARA DETECÇÃO DA COR VERMELHO

int detecVermelho(const Mat& src){
  Moments MomentosR = moments(src);
  // Cálculo da área da imagem com cor vermelha a partir do momento
  double dAreaR = MomentosR.m00;
  // Condição para determinar leitura do vermelho
  if (dAreaR>7000000){
    return 1;
  }
  waitKey(10);
  return 0;
}