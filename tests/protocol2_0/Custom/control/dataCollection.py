import numpy as np
import time

import matplotlib.pyplot as plt
from motor.motor import *
from scipy.signal import savgol_filter
from systemID.optimize import signalFr
from systemID.optimize import signal
from systemID.optimize import endTime
from systemID.optimize import endTimeFr
timeStep=0
dirs=[1,-1]

def dynamicTest(id,com):
    dyn1=motor(com,4000000)
    dyn1.connect([id])
    dyn1.setPWMControlMode(id)
    
    dyn1.setHome(id)
    dyn1.enableTorque(id) 
    
   
    
    allData=[]#list of arrays first for positive direction, second for negative direction
    s=time.time()
    loopEndtime=time.perf_counter()
    presentTime=0
    
    for dir in dirs:
        timeLs=[]
        dirData=[]
        while presentTime<=endTimeFr:
            loopStartTime=time.perf_counter()
            if(loopStartTime-loopEndtime>=timeStep):
                u=signalFr(presentTime)[0]*dir
                dyn1.setGoalPWM(u,id)
                data=dyn1.readBulkSensors(id)
                dirData.append(data)
                timeLs.append(presentTime)
                loopEndtime=time.perf_counter()
                presentTime=time.time()-s    
        dyn1.setGoalPWM(0,id)
        dyn1.disableTorque(id)
        data=np.array(dirData)
        timeArr=np.array(timeLs).reshape((len(timeLs),1))
        dataConv=data*motor.bulkConversion
        dataConv=np.append(dataConv, timeArr, axis=1)#format[[omega,time],[w,t]...]
        allData.append(post(dataConv))
    dyn1.disableTorque(id)
    return allData

def stabletest(id,com):
    dyn1=motor(com,4000000)
    dyn1.connect([id])
    dyn1.setVelocityControlMode(id)
    dyn1.setHome(id)
    dyn1.enableTorque(id)
    allData=[]#list points[(friction Torque,Omega)....] for positive and negative directions 






    
    
def post(array,plot=False):
    data=array
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
    if(plot):
        plt.plot(t,omega)
        plt.plot(tn,filteredNew)
        plt.show()
    return np.append(filteredNew,tn,axis=1)

    

if __name__ == '__main__':  
    a=post("dataFrConv.npy")
    np.save("dataFrFiltered.npy",a)