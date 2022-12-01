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
dt=0.00001

c=[B,Kt/J,Ra/La,Ke/La,1/La]#,Tcr/Jr,(Tsr-Tcr)/Jr,alphar]
K1,K2,K3,K4,K5=c



def sign(n):
    return int(n>0) - int(n<0)

class luGreModel(object):
    def __init__(self,sigma0,sigma1,sigma2,Ts,Tc,Vs,dt):
        self.sigma0=sigma0
        self.sigma1=sigma1
        self.sigma2=sigma2
        self.Ts=Ts
        self.Tc=Tc
        self.Vs=Vs
        self.z=0
        self.zDot=0
        self.dt=dt

    def calcStribertEffect(self,omega):
        r=self.Tc+(self.Ts-self.Tc)*math.e**-((omega/self.Vs)**2)
        return r/self.sigma0

    def calcStribertEffect2(self,omega):
        if(omega==0):
            hP=self.calcStricbertEffect2(self,0.001)
            hM=self.calcStricbertEffect2(self,-0.001)
            return (hP+hM)/2
        else:
            return self.calcStrickbertEffect(omega)


    def calcZDot(self,omega,z):
        g=self.calcStribertEffect(omega)
        zDot=omega-(abs(omega)/g)*z
        return zDot



    def caclTorque(self,omega):
        self.zDot=self.calcZDot(omega,self.z)
        Tf=self.sigma0*self.z+self.sigma1*self.zDot+self.sigma2*omega
        self.z=self.z+(self.zDot*self.dt)
        
        return Tf


def sensorSim(omega):
    return (omega//0.02391)*0.02391
Dmotor=13.4*10**-3

def sat(sig,threshold):
    if sig>threshold:
        return threshold
    elif(sig<-threshold):
        return -threshold
    return sig

friction=luGreModel(sigma0,sigma1,sigma2,Ts,Tc,Vs,dt)
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
    
    if(abs(currentTime-(cont)*timeStep)<=tolerance or True):
        cont+=1
        dataOmega.append(omega)
        times.append(currentTime)

    #Tf=friction.caclTorque(omega)
    omegaDot=-K1*omega+K2*i#-Tf/J
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