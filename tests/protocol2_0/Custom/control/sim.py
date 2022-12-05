from dataCollection import signal
from dataCollection import timeStep
import numpy as np
import matplotlib.pyplot as plt
import math
from dataCollection import endTime


Ra=8.1
La=0.28*10**-3

J=5.41*10**-8
Ke=0.0102469
Kt=10.2*10**-3 
B=3.121*10**-7

sigma0=100
sigma1=1.5
sigma2=0.004
Tc=0.023*Ke
Ts=0.058*Ke
Vs=6.106*10**3
dt=0.000000001

c=[B,Kt/J,Ra/La,Ke/La,1/La]
K1,K2,K3,K4,K5=c



def sign(n):
    return int(n>0) - int(n<0)



def sensorSim(omega):
    return (omega//0.02391)*0.02391


def sat(sig,threshold):
    if sig>threshold:
        return threshold
    elif(sig<-threshold):
        return -threshold
    return sig


times=[]
tolerance=0.0001
currentTime=0
omegaL=[0]
iL=[0]
omegaDotL=[]
iDotL=[]


dataOmega=[]
datau=[]
cont=0

while (currentTime<=(endTime+tolerance)):
    

    u=signal(currentTime)

    omega=omegaL[-1]
    i=iL[-1]
    
    if(abs(currentTime-(cont)*timeStep)<=tolerance or True):
        cont+=1
        dataOmega.append(omega)
        times.append(currentTime)

   
    omegaDot=-K1*omega+K2*i
    iDot=-K4*omega-K3*i+u*K5

    omegaDotL.append(omegaDot)
    iDotL.append(iDot)

    
    omegaL.append(omega+dt*omegaDot)
    iL.append(i+dt*iDot)
    currentTime+=dt

a=np.load("dataFiltered.npy")
b=np.load("dataFrFiltered.npy")


omegaSim=np.array(dataOmega)





plt.plot(times,omegaSim)
plt.plot(a[:,0],a[:,1])
plt.plot(b[:,0],b[:,1])
#plt.plot(times[:-1],uL[:-1])
plt.show()
print(c)