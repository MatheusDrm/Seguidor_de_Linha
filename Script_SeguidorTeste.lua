# Script_Teste (Bolinha)

function sysCall_init()
    -- Inicialização
    lsensor=sim.getObjectHandle('leftSensor')
    msensor=sim.getObjectHandle('middleSensor')
    rsensor=sim.getObjectHandle('rightSensor')
    lmotor=sim.getObjectHandle('leftMotor')
    rmotor=sim.getObjectHandle('rightMotor')
    camera=sim.getObjectHandle('camera')

    proximitySensor=sim.getObjectHandle('ultrasonic')

    vMotor = 10
    contadorVolta=0
    auxiliar=true
    cruzamento=false
    parar=false
    auxTempo=true
    sim.setJointTargetVelocity(lmotor, vMotor)
    sim.setJointTargetVelocity(rmotor, vMotor)

  --Função para fazer curvas
function virar(volta)
    cruzamento=true
    if (volta == 1 and datam[14]>0.5) then
          vMotorl = -10
               vMotorr = 9
               sim.setJointTargetVelocity(lmotor, vMotorl)
               sim.setJointTargetVelocity(rmotor, vMotorr)
     --Curva para Esquerda
           elseif (volta == 2 and datam[14]>0.5) then
               vMotorl = 9
               vMotorr = -9
               sim.setJointTargetVelocity(lmotor, vMotorl)
               sim.setJointTargetVelocity(rmotor, vMotorr)
           end
end

  --Função para seguir linha   
function sensor()
    if not parar then
            if (datal[11]>0.5 and datar[11]>0.5 and cruzamento==false) then
                sim.setJointTargetVelocity(lmotor, vMotor)
                sim.setJointTargetVelocity(rmotor, vMotor)
            end
            if (datal[11]>0.5 and datar[11]<0.5 and cruzamento==false) then
                sim.setJointTargetVelocity(lmotor, vMotor*1.3)
                sim.setJointTargetVelocity(rmotor, vMotor*0.3)
            end
           if (datal[11]<0.5 and datar[11]>0.5 and cruzamento==false) then
                sim.setJointTargetVelocity(lmotor, vMotor*0.3)
                sim.setJointTargetVelocity(rmotor, vMotor*1.3)
            end
        end
    end
end

function sysCall_actuation()
end

function sysCall_sensing()

    resultl,datal=sim.readVisionSensor(lsensor)
    resultm,datam=sim.readVisionSensor(msensor)
    resultr,datar=sim.readVisionSensor(rsensor)
    resultc,datac=sim.readVisionSensor(camera)

    -- LER SENSORES
    if datam[11]<0.5 then
       cruzamento=false
       vMotor=8
    end

    sensor()
    
   --sensor de proximidade
   result,distance,data=sim.readProximitySensor(proximitySensor)

   if distance ~= nil then
        if distance < 0.22 then

            --Reconhecer cor Azul

            if datac[14] > 0.4 then
                if auxiliar==true then
                    vMotor=0
                    contadorVolta=contadorVolta+1
                    auxiliar=false
                    virar(contadorVolta)
                end
            end

            -- Reconhecer cor Vermelha 

            if datac[12] > 0.3 then
                                auxiliar=true
                if auxTempo==true then
                    tAntes=sim.getSimulationTime()
                    auxTempo=false
                end
                print('tempo ', sim.getSimulationTime()-tAntes)
                if (sim.getSimulationTime()-tAntes<5.0) then
                    vMotor=0
                    sim.setJointTargetVelocity(lmotor, vMotor)
                    sim.setJointTargetVelocity(rmotor, vMotor)
                    parar=true

                else
                    parar=false
                    vMotor=7
                    sensor()
                end

            end
        end
   end
end




function sysCall_cleanup()
    
end

