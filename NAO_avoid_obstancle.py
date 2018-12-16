# -*- encoding: UTF-8 -*- 

import math
import sys
import vrep
from naoqi import ALProxy
from manage_joints import get_first_handles,JointControl
from multiprocessing import Process
import threading
from time import sleep
import time
import numpy as np

def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints

    proxy.setSmartStiffnessEnabled(False)

    proxy.rest()
    proxy.wakeUp()

    proxy.setFallManagerEnabled(True)
    #print proxy.getSummary()

        
class Robot:
    def __init__(self, motionProxy, vrepclientID):
        self.mp = motionProxy
        self.vrepclientID = vrepclientID
        self.firstOrVerification = True
        self.firstOrBuffered = True

    def WalkConfigs(self):
        #####################
        ## FOOT CONTACT PROTECTION
        #####################
        self.mp.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

        #####################
        ## Enable arms control by move algorithm
        #####################
        self.mp.setMoveArmsEnabled(True, True)
        #time.sleep(5)

    def Move(self, speed):
        ####################
        ## make robot to walking
        ####################
        dist, dist1, dist2 = self.getSensor()
        if(dist <= 0.6 and dist >0 ):  #turn around according to sensor data
            if(dist1 >= dist2):
                self.TurnArround('R')
            elif(dist1 < dist2):
                self.TurnArround('L')
        else:
            self.mp.moveTo(speed, 0, 0, self.mp.getMoveConfig("Default"))

    def GetRawOrientation(self):
        #####################
        ## get robot position in the WORLD_FRAME
        #####################
        retries = 0
        while True:
            returnCode, NaoHandle = vrep.simxGetObjectHandle(self.vrepclientID,'NAO',vrep.simx_opmode_oneshot_wait)
            retries += 1
            if (returnCode == 0 and NaoHandle is not None) or retries > 3:
                break
            retries = 0
        if returnCode == 0:
            while True:
                if self.firstOrVerification:
                    returnCode, self.RawOrientation =vrep.simxGetObjectOrientation(clientID, NaoHandle ,-1 , vrep.simx_opmode_streaming )
                    self.firstOrVerification = False
                else:
                    returnCode, self.RawOrientation =vrep.simxGetObjectOrientation(clientID, NaoHandle ,-1 , vrep.simx_opmode_buffer )
                retries += 1
                if (returnCode == 0 and self.RawOrientation is not None) or retries > 3:
                    break
        return self.RawOrientation

    def TurnArround(self, direction):
        #####################
        ## make robot turn 90 degree(not very precise)
        #####################
        currentTheta = self.GetRawOrientation()
        if direction == 'R':
            targetTheta = currentTheta[2] - math.pi/4
        elif direction == 'L':
            targetTheta = currentTheta[2] + math.pi/4

        #theta must in -pi~pi
        if targetTheta > math.pi:
            targetTheta -= math.pi
        if targetTheta < -math.pi:
            targetTheta += math.pi

        print 'curr theta %s' % currentTheta
        deltaTheta = targetTheta - currentTheta[2]
        while True:
            speed = 0.3
            if deltaTheta < 0:
                speed = -speed
            self.mp.moveToward(0, 0, speed, self.mp.getMoveConfig("Default"))
            currentTheta = self.GetRawOrientation()
            deltaTheta = targetTheta - currentTheta[2]
            print 'delta %s' %deltaTheta
            if abs(deltaTheta) < 0.04:
                break

    def getSensor(self):
        #####################
        ## return distance detected by proximity sensor 
        #####################
        retries = 0
        while True:
            returnCode1, sonarHandle1 = vrep.simxGetObjectHandle(self.vrepclientID,'Proximity_sensor1',vrep.simx_opmode_oneshot_wait)
            returnCode2, sonarHandle2 = vrep.simxGetObjectHandle(self.vrepclientID,'Proximity_sensor2',vrep.simx_opmode_oneshot_wait)
            retries += 1
            if (returnCode1 == 0 and returnCode2 == 0) or retries > 3:
                break
            retries = 0
        dist1 = -1
        dist2 = -1
        if returnCode1 == 0 and returnCode2 == 0:
            while True:
                if self.firstOrBuffered:
                    returnCode1,_,detectedPoint1,_,_=vrep.simxReadProximitySensor(self.vrepclientID,sonarHandle1,vrep.simx_opmode_streaming)
                    returnCode2,_,detectedPoint2,_,_=vrep.simxReadProximitySensor(self.vrepclientID,sonarHandle2,vrep.simx_opmode_streaming)
                    #do not use initializing value from sensor
                    dist1 = 1
                    dist2 = 1
                else:
                    returnCode1,_,detectedPoint1,_,_=vrep.simxReadProximitySensor(self.vrepclientID,sonarHandle1,vrep.simx_opmode_buffer)
                    returnCode2,_,detectedPoint2,_,_=vrep.simxReadProximitySensor(self.vrepclientID,sonarHandle2,vrep.simx_opmode_buffer)
                    dist1 = np.linalg.norm(np.array(detectedPoint1))
                    dist2 = np.linalg.norm(np.array(detectedPoint2))
                retries += 1
                if (returnCode1 == 0 and returnCode2 == 0) or retries > 3:
                    break
        dist = min(dist1, dist2)
        return dist, dist1, dist2

    def Stop(self):
        self.mp.stopMove()

class StoppableThread (threading.Thread):
    def __init__(self, clientID, motionProxy):
        super(StoppableThread, self).__init__()
        self.clientID = clientID
        self.motionProxy = motionProxy
        self.exitflag = threading.Event()
    def run(self):
        print "================ Handles Initialization ================"
        Head_Yaw=[];        Head_Pitch=[];
        L_Hip_Yaw_Pitch=[]; L_Hip_Roll=[];      L_Hip_Pitch=[]; L_Knee_Pitch=[];    L_Ankle_Pitch=[];    L_Ankle_Roll=[];
        R_Hip_Yaw_Pitch=[]; R_Hip_Roll=[];      R_Hip_Pitch=[]; R_Knee_Pitch=[];    R_Ankle_Pitch=[];    R_Ankle_Roll=[];
        L_Shoulder_Pitch=[];L_Shoulder_Roll=[]; L_Elbow_Yaw=[]; L_Elbow_Roll=[];    L_Wrist_Yaw=[]
        R_Shoulder_Pitch=[];R_Shoulder_Roll=[]; R_Elbow_Yaw=[]; R_Elbow_Roll=[];    R_Wrist_Yaw=[]
        R_H=[];             L_H=[];             R_Hand=[];      L_Hand=[];
        Body = [Head_Yaw,Head_Pitch,L_Hip_Yaw_Pitch,L_Hip_Roll,L_Hip_Pitch,L_Knee_Pitch,L_Ankle_Pitch,L_Ankle_Roll,R_Hip_Yaw_Pitch,
                R_Hip_Roll,R_Hip_Pitch,R_Knee_Pitch,R_Ankle_Pitch,R_Ankle_Roll,L_Shoulder_Pitch,L_Shoulder_Roll,L_Elbow_Yaw,L_Elbow_Roll,
                L_Wrist_Yaw,R_Shoulder_Pitch,R_Shoulder_Roll,R_Elbow_Yaw,R_Elbow_Roll,R_Wrist_Yaw,L_H,L_Hand,R_H,R_Hand]
    
        get_first_handles(clientID,Body)

        while(vrep.simxGetConnectionId(clientID)!=-1):
            if self.exitflag.isSet():
                print 'End of simulation'
                break

            JointControl(self.clientID,self.motionProxy,0,Body)

    def SetExitFlag(self):
        self.exitflag.set()
        
def startingComunication(clientID, motionProxy):
    try:
        thread1 = StoppableThread(clientID,motionProxy)
        thread1.start()

    except:
       print "Error: unable to start process"
       sys.exit(1)
    return thread1

def main(robotIP,clientID):
    try:
        motionProxy = ALProxy("ALMotion", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALMotion"
        print "Error was: ", e
        exit(1)

    # Set NAO in stiffness On
    StiffnessOn(motionProxy)

    nao = Robot(motionProx,clientID)

    thread1 = startingComunication(clientID, motionProxy)
    time.sleep(2)
    
    nao.WalkConfigs()

    print '========== NAO is listening =========='
    t = time.time()
    while(time.time()-t)<180:
        nao.Move(0.3)
    nao.Stop()

    print "================ Closing Comunication ================"
    thread1.SetExitFlag()
    thread1.join()



if __name__ == "__main__":
    robotIp = "127.0.0.1"

    print '================ Program Sarted ================'
    
    vrep.simxFinish(-1)
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
    if clientID!=-1:
        print 'Connected to remote API server'
    
    else:
        print 'Connection non successful'
        sys.exit('Could not connect')
    main(robotIp, clientID)
    print "================ Exiting Session ================"
    
    vrep.simxFinish(-1)
