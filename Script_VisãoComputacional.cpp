// Bibliotecas utizadas no OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

//Declaração de funções para detecção de cores
int detecAzul(const Mat& src);

int detecVermelho(const Mat& src);


// Dentro da int main(), mas fora do looping do while

    // Definição dos ranges de Hue, saturação e Value para as cores vermelho e azul
    
    Scalar blueLow = Scalar(100, 150, 150);
    Scalar blueHigh = Scalar(140, 255, 255);

    Scalar redLow = Scalar(0, 70, 50);
    Scalar redHigh = Scalar(10, 255, 255);
    
    //Variável para estado de detecção das cores

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
    
    // Dentro do looping while (simxGetConnectionId(clientID) != -1): 
    
    // Receber imagem da camera
    if(simxGetVisionSensorImage(clientID,sensorHandle[5],res,&image,1,simx_opmode_streaming) == simx_return_ok){
        //printf("Camera reconhecida");
      }else{
        //printf("Camera nao reconhecida");
      }
      
    // Código base do processamento de imagem
    
    if(simxReadVisionSensor(clientID,sensorHandle[5], detectionState, &auxValues, &auxValuesCount,simx_opmode_streaming) == simx_return_ok){
        // Essa função retorna um vetor auxValues em que o 14 termo é a profundidade medida pela câmera e, portanto, será utilizada para medir a distância à objetos
          //                                CÓDIGO BLOCO AZUL
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
          if (auxValues[14]<0.048 && cruzamento==0 && azul==1){
            cout << "entrou curva";
            simxSetJointTargetVelocity(clientID, leftMotorHandle, 5, simx_opmode_streaming);
            simxSetJointTargetVelocity(clientID, rightMotorHandle, 5, simx_opmode_streaming);
            if (auxValues[14]<0.035){
              cout << "entrou curva 2" << endl;
              cruzamento = 1;
              simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, simx_opmode_streaming);
              simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, simx_opmode_streaming);
              curva();       
            }
            parar = 1;
            saida = 0;
          }
          
          //                               CÓDIGO BLOCO VERMELHO
          
         if (auxValues[14]<0.035 && vermelho==1){
              if (auxTempo==1){
                auxTempo=0;;
                tempo();
              }
            //parar = 1;
            //saida = 0;
            }
            
      }else{
        printf("Problemas para pegar imagem");
      }
