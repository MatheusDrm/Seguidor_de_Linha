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

// Declaração das funções utilizadas necessárias

void func_P(void);

// Dentro da int main(){
	
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

