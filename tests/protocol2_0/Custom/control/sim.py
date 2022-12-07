import systemID.simulations.Model as Models
import time
import scipy.integrate as ODE
from scipy.signal import resample
from scipy.signal import resample_poly
import numpy as np
import matplotlib.pyplot as plt
import math

endTime=10

Ra=8.1
La=0.28*10**-3

J=5.41*10**-8
Ke=0.0102469
Kt=10.2*10**-3 
B=3.121*10**-7

sigma0=0.1
sigma1=0.1
sigma2=1
Tc=0.023*Ke
Ts=0.058*Ke
Vs=6.106*10**3
r=(22.28/2)*10**-3


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

def signal(t):# t in seconds [0 10]
    return (3*math.sin((math.pi/endTime)*t),0)


motor=Models.motorFrictionModel(Ra,La,J,Kt,B,sigma0,sigma1,sigma2,Ts,Tc,Vs,r,signal,[0,0,0])
motor.sim()
a=len(motor.y[0])


data=np.load("dataConv.npy")
timeS=data[:,-1]
omega=data[:,2]
#plt.plot(timeS,omega)
p=len(omega)
newdata=resample(motor.y[0],p,motor.t,domain='time')

plt.plot(newdata[1],newdata[0])
#plt.plot(motor.t,motor.y[0])

plt.show()

