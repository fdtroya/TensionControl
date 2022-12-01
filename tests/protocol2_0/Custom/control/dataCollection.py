import numpy as np
import time
import sys, os
import math

from motor.motor import *

timeStep=0.00 
endTime=9


def signal(t):# t in seconds [0 10]
    return 3*math.sin((math.pi/endTime)*t)

#position control


id=2
def runTest(): 
    dyn1=motor('COM3',4000000)
    dyn1.connect([id])
    dyn1.setPWMControlMode(id)
    # Enable Dynamixel Torque 
    dyn1.setHome(id)
    dyn1.enableTorque(id) 
    pos=0
    timeLs=[]
    allData=[]
    uL=[]
    s=time.time()
    loopEndtime=time.perf_counter()
    presentTime=0
    c=0

    while presentTime<=endTime:
        loopStartTime=time.perf_counter()
        if(loopStartTime-loopEndtime>=timeStep):
            u=signal(presentTime)
            dyn1.setGoalPWM(u,id)
            data=dyn1.readBulkSensors(id)
            allData.append(data)
            timeLs.append(presentTime)
            
            c+=1
            loopEndtime=time.perf_counter()
            presentTime=time.time()-s
            
    dyn1.setGoalPWM(0,id)
    dyn1.disableTorque(id)

    data=np.array(allData)
    timeArr=np.array(timeLs).reshape((len(timeLs),1))
    
    
    dataConv=data*motor.bulkConversion
    
    dataConv=np.append(dataConv, timeArr, axis=1)

    np.save("dataConv.npy",dataConv)
    
    
    

if __name__ == '__main__':  
    runTest()