import numpy as np
import time
import sys, os
import math

from motor.motor import *

def signal(t):# t in seconds [0 10]
    Kda=10
    return(700/Kda)*math.sin((math.pi/10)*t)

#position control


dyn1=motor(1,'COM3',4000000)
dyn1.connect()
dyn1.setPWMControlMode()
# Enable Dynamixel Torque 
dyn1.setHome()
dyn1.enableTorque() 
pos=0
timeLs=[]
allData=[]
timestart=time.time()
presentTime=time.time()
while (presentTime-timestart)<10:
    presentTime=time.time()
    t=presentTime-timestart
    u=signal(t)
    dyn1.setGoalPWM(u)
    data=dyn1.readBulkSensors()
    allData.append(data)
    timeLs.append(t)

dyn1.setGoalPWM(0)
dyn1.disableTorque()
data=np.array(allData)
dataConv=data*motor.bulkConversion
timeArr=np.array(timeLs)
np.save("dataConv.npy",dataConv)
np.save("timeStep.npy",timeArr)


