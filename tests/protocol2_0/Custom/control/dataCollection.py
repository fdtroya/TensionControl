import numpy as np
import time
import sys, os
import math

from motor.motor import *

timeStep=0.005
endTime=10

def signal(t):# t in seconds [0 10]
    Kda=10
    return(700/Kda)*math.sin((math.pi/10)*t)

#position control



def runTest(): 
    dyn1=motor(1,'COM3',4000000)
    dyn1.connect()
    dyn1.setPWMControlMode()
    # Enable Dynamixel Torque 
    dyn1.setHome()
    dyn1.enableTorque() 
    pos=0
    timeLs=[]
    allData=[]
    uL=[]
    loopEndtime=time.perf_counter()
    presentTime=0
    while presentTime<=endTime:
        loopStartTime=time.perf_counter()
        if(loopStartTime-loopEndtime>=timeStep):
            u=signal(presentTime)
            dyn1.setGoalPWM(u)
            data=dyn1.readBulkSensors()
            allData.append(data)
            timeLs.append(presentTime)
            uL.append(u)
            presentTime+=timeStep
            loopEndtime=time.perf_counter()


    dyn1.setGoalPWM(0)
    dyn1.disableTorque()
    data=np.array(allData)
    timeArr=np.array(timeLs)
    uL=np.array(uL)
    dataConv=data*motor.bulkConversion

    np.save("dataConv.npy",dataConv)
    np.save("timeStep.npy",timeArr)
    np.save("u.npy",uL)


