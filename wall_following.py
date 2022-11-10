import time # time library is used to pause the sensors for 0.2 seconds.
import random # random library is used to get random numbers for wandering .
import sim as vrep # sim library is used to connect the vrep with python.
import numpy as np # numpy software is used to determine the magnitude of the (x,y,z) 3D vector.


class PioneerRobot(): # Class name for the robot
    def __init__(self):
        # Create the objects to call the motors
        returnCode, self.rightMotor = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_blocking)
        returnCode, self.leftMotor = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_blocking)
        # Create the objects to call the sonic sensors
        returnCode, self.sensorOne = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor1", vrep.simx_opmode_blocking)
        returnCode, self.sensorTwo = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor2",vrep.simx_opmode_blocking)
        returnCode, self.sensorThree = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor3", vrep.simx_opmode_blocking)
        returnCode, self.sensorFour = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor4",vrep.simx_opmode_blocking)
        returnCode, self.sensorFive = vrep.simxGetObjectHandle(clientID,"Pioneer_p3dx_ultrasonicSensor5", vrep.simx_opmode_blocking)
        returnCode, self.sensorSix = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor6", vrep.simx_opmode_blocking)
        returnCode, self.sensorSeven = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor7", vrep.simx_opmode_blocking)
        returnCode, self.sensorEight = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor8",vrep.simx_opmode_blocking)
        returnCode, self.sensorNine = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor9",vrep.simx_opmode_blocking)
        returnCode, self.sensorTen = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor10",vrep.simx_opmode_blocking)
        returnCode, self.sensorEleven = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor11",vrep.simx_opmode_blocking)
        returnCode, self.sensorTwelve = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor12",vrep.simx_opmode_blocking)
        returnCode, self.sensorThirteen = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor13",vrep.simx_opmode_blocking)
        returnCode, self.sensorFourteen = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor14",vrep.simx_opmode_blocking)
        returnCode, self.sensorFifteen = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor15",vrep.simx_opmode_blocking)
        returnCode, self.sensorSixteen = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor16",vrep.simx_opmode_blocking)
        # Create the objects to call the graph and dummy object
        returnCode, self.graph = vrep.simxGetObjectHandle(clientID,'Graph', vrep.simx_opmode_blocking)
        returnCode, self.dummy = vrep.simxGetObjectHandle(clientID,"Dummy",vrep.simx_opmode_blocking)
        # First read of the sonar sensors
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorOne, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorTwo, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorThree, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorFour, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorFive, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorSix, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorSeven, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorEight, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorNine, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorTen, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorEleven, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorTwelve, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorThirteen, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorFourteen, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorFifteen, vrep.simx_opmode_streaming)
        returnCode = vrep.simxReadProximitySensor(clientID, self.sensorSixteen, vrep.simx_opmode_streaming)


    def checkCollision(self): #Function required for the robot's collision status
        returnCode, collisionState = vrep.simxCheckCollision(clientID, "Pioneer_p3dx","80cmHighWall200cm_visible7", vrep.simx_opmode_buffer)
        if collisionState == 1:
            return "Collusion detected"
        else:
            return 0

    def getObjectPosition(self): # To measure the object on the Pioneer robot
       returnCode, position = vrep.simxGetObjectPosition(clientID,self.dummy,-1,vrep.simx_opmode_blocking)


    def distanceToWall(self, frontMidSensor):
        returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, frontMidSensor, vrep.simx_opmode_buffer)
        x = np.array(detectedPoint)
        if detectionState == True:
            return np.linalg.norm(x) # using numpy software to get a magnitude from detectedPoint
        else:
            return 100 # It should be an interger bigger than that magnitude. So, i go with 100. It can be any other integer number

    def startRobot(self): #Function required for the robot to start
        returnCode = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, 1, vrep.simx_opmode_blocking)
        returnCode = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, 1, vrep.simx_opmode_blocking)

    def stopRobot(self): #Function required for the robot to stop
        returnCode = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, 0, vrep.simx_opmode_blocking)
        returnCode = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, 0, vrep.simx_opmode_blocking)

    def turnRobot(self, turnRight, turnLeft): #Function required for the robot to turn
        returnCode = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, turnRight, vrep.simx_opmode_blocking)
        returnCode = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, turnLeft, vrep.simx_opmode_blocking)

    def wander(self): # Function required for the robot to wander
        a = random.randint(0,10)
        b = random.randint(0,10)
        returnCode = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, a, vrep.simx_opmode_blocking)
        returnCode = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, b, vrep.simx_opmode_blocking)



vrep.simxFinish(-1) # closes the previous
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # coordinates for connection with vrep and python
if clientID != -1: # if there is connection with vrep
    print('Connected to remote API server') # prints if the connection is successful
    robot = PioneerRobot() #We're changing the class name to shorter name to make things easier.

    count = 0
    while (count < 10):
        count = count + 1 # prints the distance to wall for each sensor for 10 times and puases for 0.2 seconds
        print(robot.distanceToWall(robot.sensorOne))
        print(robot.distanceToWall(robot.sensorTwo))
        print(robot.distanceToWall(robot.sensorThree))
        print(robot.distanceToWall(robot.sensorFour))
        print(robot.distanceToWall(robot.sensorFive))
        print(robot.distanceToWall(robot.sensorSix))
        print(robot.distanceToWall(robot.sensorSeven))
        print(robot.distanceToWall(robot.sensorEight))
        print(robot.distanceToWall(robot.sensorNine))
        print(robot.distanceToWall(robot.sensorTen))
        print(robot.distanceToWall(robot.sensorEleven))
        print(robot.distanceToWall(robot.sensorTwelve))
        print(robot.distanceToWall(robot.sensorThirteen))
        print(robot.distanceToWall(robot.sensorFourteen))
        print(robot.distanceToWall(robot.sensorFifteen))
        print(robot.distanceToWall(robot.sensorSixteen))
        time.sleep(0.2) # pauses for 0.2 seconds
    a = 1
    while a < 2:
        robot.startRobot() # Activates the startRobot function and the robot starts to move

#        robot.checkCollision()
        robot.getObjectPosition() # Gets the position of the robot

        if robot.distanceToWall(robot.sensorOne) < 0.8: #Threshold Value
            robot.turnRobot(2.1,-2.1) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorTwo) < 0.8:  #Threshold Value
            robot.turnRobot(2,-2) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorThree) < 0.8: #Threshold Value
            robot.turnRobot(1.9,-1.9) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorFour) < 0.8: #Threshold Value
            robot.turnRobot(2,-2) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorFive) < 0.8: #Threshold Value
            robot.turnRobot(2.1,-2.1) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorSix) < 0.8: #Threshold Value
            robot.turnRobot(0.1,0.2) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorSeven) < 0.8: #Threshold Value
            robot.turnRobot(0.2,0.1) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorEight) < 0.4: #0.3 #Threshold Value
            robot.turnRobot(0.3,0.1) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorEight) < 0.8: #Threshold Value
            robot.turnRobot(0.1,0.4) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorNine) < 1.8:  #Threshold Value
            robot.turnRobot(0.1,1.0) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorTen) < 0.8: #Threshold Value
            robot.turnRobot(0.2,1.1) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorTwelve) < 0.8: #Threshold Value
            robot.turnRobot(0.3,1.2) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorThirteen) < 0.8: #Threshold Value
            robot.turnRobot(0.4,1.3) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorFourteen) < 1.8: #Threshold Value
            robot.turnRobot(1.0,0.1) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorFifteen) < 0.4:  #Threshold Value
            robot.turnRobot(0.1,0.3) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorFifteen) < 1.8:  #Threshold Value
            robot.turnRobot(0.4,0.1) # Velocity of right and left motors

        elif robot.distanceToWall(robot.sensorSixteen) < 0.8: #Threshold Value
            robot.turnRobot(0.5,0.2) # Velocity of right and left motors
        else:
            robot.wander() # Activates the Wander function and robot wanders around

    robot.stopRobot() # Activates the stopRobot function and stops the robot


    vrep.simxGetPingTime(clientID)
    
    vrep.simxFinish(clientID) # Finishes the connection with VREP
else:
    print('Failed connecting to remote API server')
print('Program ended')