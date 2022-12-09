import systemID.simulations.Model as Model
import time
from scipy import interpolate
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


guess=[8.71427011e+00 ,2.69348163e-04 ,1.30802543e-03 ,2.66204421e+00,3.94582734e-01]
guessFr=[0.02548752 ,0.01175936, 0.00247281, 1.10968817 ,0.39299388 ,0.47506953]

Ra,La,J,Kt,B=guess
sigma0,sigma1,sigma2,Ts,Tc,Vs=guessFr





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

constants=[Ra, La, J, Kt, B,sigma0,sigma1,sigma2,Ts,Tc,Vs]
motor=Model.motorFrictionModel(constants=constants,r=r,initialState=[0,0,0],inputFunction=signal)

a=len(motor.y[0])

f=interpolate.interp1d(motor.t, motor.y[0],fill_value=0,bounds_error=False)
ts=np.arange(0,10,0.001)


plt.plot(motor.t,motor.y[0])
plt.plot(ts,f(ts))

plt.show()

