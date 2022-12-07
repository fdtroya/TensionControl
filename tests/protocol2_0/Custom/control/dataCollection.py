import numpy as np
import time
import sys, os
import math
import matplotlib.pyplot as plt
from motor.motor import *
from scipy.signal import savgol_filter
from scipy.interpolate import UnivariateSpline


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
    
def post(name):
    data=np.load(name)
    omega=data[:,2]
    t=data[:,-1]
    newOmegaData=[]
    
    repeats=0
    for w_index in range(len(omega)):
        
        if w_index==0:
            newOmegaData.append([omega[w_index],t[w_index]])
        else:
            lastO,lastT=newOmegaData[-1]
            if (lastO!= omega[w_index])or repeats>=50 :
                repeats=0
                newOmegaData.append([omega[w_index],t[w_index]])
            else:
                repeats+=1

    newOmegaData=np.array(newOmegaData)
    tn=newOmegaData[:,-1]
    filteredNew=savgol_filter(newOmegaData[:,0], 5, 3)
    tn=tn.reshape((len(tn),1))
    filteredNew=filteredNew.reshape((len(filteredNew),1))
    plt.plot(t,omega)
    plt.plot(tn,filteredNew)
    plt.show()


    
    
    return np.append(filteredNew,tn,axis=1)

    

if __name__ == '__main__':  
    a=post("dataConv.npy")
    np.save("dataHFiltered.npy",a)