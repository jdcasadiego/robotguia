#---------------------------------------------------------
#autor: Jose David Casadiego Lopez
#e-mail: jd.casadiego10@uniandes.edu.co
#
#Requerimientos:
# - Ubuntu 16.04 o mayor
# - CoppeliaSim https://www.coppeliarobotics.com/
#       - Tener la escena P3-DX_sync_guide_test.ttt abierta en CoppeliaSim
# - OpenPose https://github.com/CMU-Perceptual-Computing-Lab/openpose
#---------------------------------------------------------

#---------- Dependencias importadas ----------
import b0RemoteApi
import math
import time
import os
import json

#Declaración del nodo B0 para la comunicación con CoppeliaSim
with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi') as client:
    #---------- Constantes y variables ----------
    #Variables de simulación
    client.doNextStep=True
    client.runInSynchronousMode=True
    client.simuTime=0           #Tiempo de simulación
    client.statusId='notReady'  #Estado del simulador. Habilitar para módo de operación asíncrono
    
    #Variables de procesamiento de imágenes
    client.frameNumber=0    #contador de frames procesados
    client.puntos=[]        #puntos detectados por OpenPose
    client.neckX=0          #posición en x del cuello
    client.lostFrames = 0   #contador de frames para determinar si la cámara ve a la persona
    middlePixel = 90        #pixel de control
    waitPixel = 40          #pixel de espera

    #Variables del robot
    client.maxVel = 0.6     #m/s
    client.turnSpeed = 0.3  #rad/s
    R = 0.0975              #radio de las ruedas
    L = 0.331               #distancia entre ruedas

    client.robotX = 0       
    client.robotY = 0       #posición y orientación. Marco de ref. global
    client.robotTheta = 0   
    Kp = 4
    Ki = 0.8*0.2            #ganancias del controlador PID
    Kd = 0.1/0.2
    client.EAcum = 0        #Error acumulado del controlador PID
    client.eAnt = 0         #Error de iteración anterior del controlador PID

    client.atGoal = False       #bandera de llegada al destino
    client.userLost = False     #bandera de usuario perdido

    #---------- Funciones ----------
    #Espera a que el simulador indique un estado determinado. Usada en modo asíncrono.
    def waitForSignal(id):
        while client.statusId!=id:
            client.simxSpinOnce()
	
    #Recibe el String del estado del simulador para cada iteración.
    def statusId_callback(msg):
        client.statusId=msg[1]

    #Recibe el tiempo de simulación al empezar el paso de simulación para cada iteración.
    def simulationStepStarted(msg):
        simTime=msg[1][b'simulationTime'];
        #print('Simulation step started. Simulation time: ',simTime) #debug
    
    #Recibe el tiempo de simulación al terminar el paso de simulación para cada iteración.
    def simulationStepDone(msg):
        simTime=msg[1][b'simulationTime'];
        #print('Simulation step done. Simulation time: ',simTime); #debug
        client.doNextStep=True

    #Recibe y actualiza el tiempo de simulación actual para cada iteración.
    def simuTime_callback(msg):
        client.simuTime=msg[1]

    #Recibe y actualiza la posición global del robot para cada iteración.
    def position_callback(msg):
        pos=msg[1]
	client.robotX = pos[0]
	client.robotY = pos[1]

    #Recibe y actualiza la posición global del robot para cada iteración.
    def orientation_callback(msg):
        orientation=msg[1]
	client.robotTheta = orientation[2]

    #Módulo de detección de personas. Procesa la imagen de la cámara del simulador y actualiza la posición y el estado de la persona vista.
    def readCameraFrame():
	ret=client.simxCallScriptFunction('takePicture@Pioneer_p3dx','sim.scripttype_childscript',client.frameNumber,client.simxDefaultPublisher()) #Revisar en CoppeliaSim la ruta donde se guardan las imagenes!
	#waitForSignal('picture') #Habilitar para módo de operación asíncrono
	os.system('cd ~/openpose && ./build/examples/openpose/openpose.bin --image_dir ~/User_models/synchro/guide_test/camera --write_json ~/User_models/synchro/guide_test/openpose_output --render_pose 0 --display 0 --logging_level 5') #Revisar rutas de entradas y salidas!
	os.system('cd ~/User_models/synchro/guide_test/camera && rm -f *.png') #Revisar ruta de las imagenes!
	with open('./openpose_output/test'+str(client.frameNumber)+'_keypoints.json') as f: #Revisar la ruta del archivo de salida de OpenPose
	    data = json.load(f)
	if(data['people']!=[]):     #Sólo actualiza los keypoints si detecta a una persona.
	    for i in data['people']:
		client.puntos = i['pose_keypoints_2d']
	    newNeck=client.puntos[3]
	    if(newNeck!=0):
		client.neckX=newNeck
		client.userLost = False
		client.lostFrames = 0
	elif(client.lostFrames<3):  #Empieza a contar frames si no detecta personas.
	    client.lostFrames = client.lostFrames + 1
	else:                       #Declara que ha perdido al usuario.
	    client.userLost = True
	client.frameNumber = client.frameNumber + 1

    #Comportamiento de ir al destino. Actualiza la velocidad de las ruedas del robot.
    def goToGoal(goalX,goalY):
	v_ref = 0.4
	v = v_ref*((client.neckX-waitPixel)/middlePixel)
	if(v>client.maxVel):
	    v = client.maxVel
	elif(v<0):
	    v = 0
	xdif = goalX - client.robotX
	ydif = goalY - client.robotY
	dist = math.sqrt(ydif**2 + xdif**2)
	#print(str(dist)) #debug
	if(dist<0.1):
	    client.atGoal=True
	ref = math.atan2(ydif,xdif)
	e = math.atan2(math.sin(ref-client.robotTheta),math.cos(ref-client.robotTheta))
	#print(str(e)) #debug
	client.EAcum = client.EAcum+e
	w = Kp*e + Kd*(e-client.eAnt) + Ki*client.EAcum
	client.eAnt = e
	vRight = (2*v + w*L)/(2*R)
	vLeft = (2*v - w*L)/(2*R)
	#print(str(client.simuTime)+', '+str(client.neckX)+', '+str(v)+', '+str(w)) #debug
	velRobot = [vLeft, vRight]
	return velRobot

    #Comportamiento de buscar al usuario. Actualiza la velocidad de las ruedas del robot.
    def lookForUser():
	turnDirection = 1
	if(client.neckX >128):
	    turnDirection = -1
	v = 0
	w = client.turnSpeed*turnDirection
	vRight = (2*v + w*L)/(2*R)
	vLeft = (2*v - w*L)/(2*R)
	#print(str(client.simuTime)+', '+str(client.neckX)+', '+str(v)+', '+str(w)) #debug
	velRobot = [vLeft, vRight]
	return velRobot

    #Algoritmo de robot guía para cada iteración
    def stepSimulation(goalX,goalY):
        if client.runInSynchronousMode:
            while not client.doNextStep:
                client.simxSpinOnce()

            client.doNextStep=False
	    readCameraFrame()
	    vel=[0, 0]
	    if(client.userLost):
		#print('looking for user') #debug
		vel = lookForUser()
	    else:
		#print('going to goal') #debug
		vel = goToGoal(goalX,goalY)
	    ret=client.simxCallScriptFunction('moveAtSpeed@Pioneer_p3dx','sim.scripttype_childscript',vel,client.simxDefaultPublisher())
            client.simxSynchronousTrigger()
        else:
            client.simxSpinOnce()
    
    #---------- Inicialización ----------
    client.simxAddStatusbarMessage('Hello world!',client.simxDefaultPublisher())
    client.simxGetStringSignal('p3dx_statusId',client.simxDefaultSubscriber(statusId_callback));
    visionSensorHandle=client.simxGetObjectHandle('Vision_sensor',client.simxServiceCall())
    leftMotorHandle=client.simxGetObjectHandle('Pioneer_p3dx_leftMotor',client.simxServiceCall())
    rightMotorHandle=client.simxGetObjectHandle('Pioneer_p3dx_rightMotor',client.simxServiceCall())
    odometryHandle=client.simxGetObjectHandle('p3dx_odometry',client.simxServiceCall())
    client.simxGetObjectPosition(odometryHandle[1],-1,client.simxDefaultSubscriber(position_callback));
    client.simxGetObjectOrientation(odometryHandle[1],-1,client.simxDefaultSubscriber(orientation_callback));

    if client.runInSynchronousMode:
        client.simxSynchronous(True)
    
    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted));
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone));
    client.simxGetSimulationTime(client.simxDefaultSubscriber(simuTime_callback));
    client.simxStartSimulation(client.simxDefaultPublisher())
    #waitForSignal('ready')     #Habilitar para modo de operación asíncrono
    
    startTime=client.simuTime
    client.goalMarker= client.simxAddDrawingObject_points(15,[0,150,0],[0,0.5,0.001],client.simxServiceCall())
    while (not client.atGoal):
	stepSimulation(0,0.5)
	#print(str(client.atGoal)) #debug
    client.simxRemoveDrawingObject(client.goalMarker[1],client.simxDefaultPublisher())

    client.atGoal = False
    client.EAcum = 0
    client.eAnt = 0
    client.goalMarker= client.simxAddDrawingObject_points(15,[0,150,0],[1.5,-1,0.001],client.simxServiceCall())
    while (not client.atGoal):
	stepSimulation(1.5,-1)
	#print(str(client.atGoal))
    client.simxRemoveDrawingObject(client.goalMarker[1],client.simxDefaultPublisher())

    client.atGoal = False
    client.EAcum = 0
    client.eAnt = 0
    client.goalMarker= client.simxAddDrawingObject_points(15,[0,150,0],[1.5,-4,0.001],client.simxServiceCall())
    while (not client.atGoal):
	stepSimulation(1.5,-4)
	#print(str(client.atGoal)) #debug
    client.simxRemoveDrawingObject(client.goalMarker[1],client.simxDefaultPublisher())

    client.simxStopSimulation(client.simxDefaultPublisher())
