-- Código do seguidor de linha em linguagem lua

function sysCall_init()
    -- Inicialização
    motorEsquerdo=sim.getObjectHandle('Roda_1')
    motorDireito=sim.getObjectHandle('Roda_2')
    sensor0=sim.getObjectHandle('sensor0')
    sensor1=sim.getObjectHandle('sensor1')
    sensor2=sim.getObjectHandle('sensor2')
    sensor3=sim.getObjectHandle('sensor3')
    sensor4=sim.getObjectHandle('sensor4')
    camera=sim.getObjectHandle('camera')

    -- Definindo valores iniciais para as variáveis

    vMotor=20
    contadorVolta=0
    auxiliar=true
    cruzamento=false
    parar=false
    auxTempo=true
    erro=0
    erroanterior=0
    
    integral = 0
    derivativo = 0
    
    kp=2
    ki=0.5
    kd=1
    
    
    sim.setJointTargetVelocity(motorEsquerdo, vMotor)
    sim.setJointTargetVelocity(motorDireito, vMotor)
end

   -- Função para realização das curvas no cruzamento
function virar(volta)
	-- Virar para a direita
    if (volta == 1 and data2[14]>0.5) then
        vMotorl = -10
        vMotorr = 5
        sim.setJointTargetVelocity(motorEsquerdo, vMotorl)
        sim.setJointTargetVelocity(motorDireito, vMotorr)
	-- Virar para a esquerda
      elseif (volta == 2 and data2[14]>0.5) then
        vMotorl = 5
        vMotorr = -10
        sim.setJointTargetVelocity(motorEsquerdo, vMotorl)
        sim.setJointTargetVelocity(motorDireito, vMotorr)
     end
end
    
    -- Função para ajuste do erro utilizado no controle PID
function erroCalc()

        if (data0[11]>0.5 and data1[11]>0.5 and data2[11]>0.5 and data3[11]>0.5 and data4[11]<0.5) then
                erro = 4
           
         elseif (data0[11]>0.5 and data1[11]>0.5 and data2[11]>0.5 and data3[11]<0.5 and data4[11]<0.5) then
                erro = 3
               
         elseif (data0[11]>0.5 and data1[11]>0.5 and data2[11]>0.5 and data3[11]<0.5 and data4[11]>0.5) then
                erro = 2
             
         elseif (data0[11]>0.5 and data1[11]>0.5 and data2[11]<0.5 and data3[11]<0.5 and data4[11]>0.5) then
                erro = 1
             
         elseif (data0[11]>0.5 and data1[11]>0.5 and data2[11]<0.5 and data3[11]>0.5 and data4[11]>0.5) then
                erro = 0
              
         elseif (data0[11]>0.5 and data1[11]<0.5 and data2[11]<0.5 and data3[11]>0.5 and data4[11]>0.5) then
                erro = -1
              
         elseif (data0[11]>0.5 and data1[11]<0.5 and data2[11]>0.5 and data3[11]>0.5 and data4[11]>0.5) then
                erro = -2
              
         elseif (data0[11]<0.5 and data1[11]<0.5 and data2[11]>0.5 and data3[11]>0.5 and data4[11]>0.5) then
                erro = -3
            
         elseif (data0[11]<0.5 and data1[11]>0.5 and data2[11]>0.5 and data3[11]>.5 and data4[11]>0.5) then
                erro = -4
            
         elseif (data0[11]>0.5 and data1[11]>0.5 and data2[11]>0.5 and data3[11]>.5 and data4[11]>0.5) then
                if (erroanterior <= -4) then
                    erro = -5  
                else
                    erro = 5
                end
        end          
end
    
    -- Função para implementação do controle PID

    function pid()
        TermoP = erro * kp
        
        integral = integral + erro
        TermoI = ki * integral
        
        derivativo = erro - erroanterior
        TermoD = derivativo * kd
        
        saida = TermoP + TermoI + TermoD
        
        erroanterior = erro
    end
    
    --Função para ajusta da velocidade dos motores de acordo com o controle PID

    function velocidade() 
        sim.setJointTargetVelocity(motorEsquerdo, vMotor + saida)
        sim.setJointTargetVelocity(motorDireito, vMotor - saida)
    end
    

function sysCall_actuation()
    
end

function sysCall_sensing()
 -- Receber dados dos sensores
    result0,data0=sim.readVisionSensor(sensor0)
    result1,data1=sim.readVisionSensor(sensor1)
    result2,data2=sim.readVisionSensor(sensor2)
    result3,data3=sim.readVisionSensor(sensor3)
    result4,data4=sim.readVisionSensor(sensor4)
    resultc,datac=sim.readVisionSensor(camera)
    distance = datac[15]

     -- LER SENSORES
    if data2[11]<0.5 then
    	cruzamento=false
    	vMotor=20
    end
   
 -- Verificar se o seguidor está no cruzamento, caso não esteja o PID será executado normalmente
    if not cruzamento then
    	erroCalc()
    	pid()
    	velocidade()
    end
 -- Em caso positivo, será necessário zerar as variáveis do controle PID para evitar erros posteriormente a curva.
    if cruzamento==true then
        erro=0
        integral = 0
        derivativo = 0
        virar(contadorVolta)
    end
   
 ----------------------------------- Tarefas da Visão Computacional----------------------------------------
   if distance ~= nil then
        if distance < 0.037 then
          -- Script Bloco Azul
            if datac[14] > 0.4 then
            	cruzamento=true
                if auxiliar==true then
                    cruzamento=true
                    vMotor=0
                    -- Ao reconhcer a cor azul soma-se 1 ao contadorVolta 
                    contadorVolta=contadorVolta+1
                    auxiliar=false
                end
            end
            -- Script Bloco Vermelho
            if datac[12] > 0.3 then
                auxiliar=true
		-- Script para obter o tempo da simulação quando o seguidor encontra o bloco vermelho
                if auxTempo==true then
                    tAntes=sim.getSimulationTime()
                    auxTempo=false
                end
                print('tempo ', sim.getSimulationTime()-tAntes)
		-- Script para desligar os motores do seguidor por 5 segundos depois de encontrar o bloco vermelho
                if (sim.getSimulationTime()-tAntes<5.0) then
                    vMotor=0
                    sim.setJointTargetVelocity(motorEsquerdo, vMotor)
                    sim.setJointTargetVelocity(motorDireito, vMotor)
                    parar=true
                    erro=0
                    integral = 0
                    derivativo = 0
		-- Script para voltar a se movimentar após os 5 segundos parado
                else
                    parar=false
                    vMotor=20
                    erroCalc()
                    pid()
                    velocidade()                    
                end
            end
        end
   end
end

function sysCall_cleanup()

end
