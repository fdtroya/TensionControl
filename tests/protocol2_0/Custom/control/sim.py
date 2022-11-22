from dataCollection import signal
from dataCollection import timeStep
import numpy as np
import matplotlib.pyplot as plt


def sensorSim(omega):
    return (omega//0.02391)*0.02391



"""
B=0.2
J=0.02
Kt=0.015
Ke=0.015
La=0.5
Ra=2
"""

Dmotor=13.4*10**-3

Ra=8.1
La=0.28*10**-3

J=5.41*10**-8
Ke=0.0102469
Kt=10.2*10**-3 

B=3.121*10**-7
dt=0.00001

c=[B/J,Kt/J,Ra/La,Ke/La,1/La]#,Tcr/Jr,(Tsr-Tcr)/Jr,alphar]
K1,K2,K3,K4,K5=c
def sat(sig,threshold):
    if sig>threshold:
        return threshold
    elif(sig<-threshold):
        return -threshold
    return sig


endTime=10
times=[]
tolerance=0.0001
currentTime=0
omegaL=[0]
iL=[0]
omegaDotL=[]
iDotL=[]
uL=[]

dataOmega=[]
datau=[]
cont=0
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

a=np.load("dataConv.npy")
#a[:,2]=a[:,2]*353.5
time=np.load("timeStep.npy")
omegaL=np.array(dataOmega)

plt.plot(times,omegaL)
experimentalOmega=a[:,2]
plt.plot(time,experimentalOmega)

#plt.plot(times[:-1],uL[:-1])
plt.show()
print(c)