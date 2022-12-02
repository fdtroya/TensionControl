from dataCollection import signal
import math 
import numpy as np
import matplotlib.pyplot as plt

def sensorSim(omega):
    return (omega//0.02391)*0.02391


def sign(n):
    return int(n>0) - int(n<0)
    
def runSim(listK,timeStep,endTime):
    currentTime=0
    cont=0
    dt=0.00001
    omegaL=[0]
    iL=[0]
    omegaDotL=[]
    iDotL=[]
    uL=[]
    K1,K2,K3,K4,K5,K6,K7,K8=listK
    times=[]
    tolerance=0.0001
    dataOmega=[]
    while (currentTime<=(endTime+tolerance)):
        
        u=signal(currentTime)
        uL.append(u)
        omega=omegaL[-1]
        i=iL[-1]

        if(abs(currentTime-(cont)*timeStep)<=tolerance):
            cont+=1
            dataOmega.append(sensorSim(omega/353.5))
            times.append(currentTime)


        omegaDot=-K1*omega+K2*i
        iDot=-K4*omega-K3*i+u*K5


        omegaDotL.append(omegaDot)
        iDotL.append(iDot)

        omegaL.append(omega+dt*omegaDot)
        iL.append(i+dt*iDot)
        currentTime+=dt
        
    omegaL=np.array(dataOmega)
    plt.plot(times,omegaL)
    plt.show()
    return uL,iL[1:],omegaL[:-1],omegaDot


