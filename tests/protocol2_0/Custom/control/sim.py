import systemID.simulations.Model as Models
import time
import scipy.integrate as ODE

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





#motor=Models.motorModel(Ra,La,J,Ke,Kt,B,signal)
motor=Models.motorFrictionModel(Ra,La,J,Ke,Kt,B,sigma0,sigma1,sigma2,Ts,Tc,Vs,r,signal)
sol=ODE.solve_ivp(motor.evalStateEq,[0,10],[0,0,0],method='LSODA')
plt.plot(sol.t,sol.y[0])
"""
methods=['RK45','RK23','DOP853','Radau','BDF','LSODA']
ys=[]
ts=[]
for m in methods:
    initial=time.time()
    sol=ODE.solve_ivp(motorModel.evalStateEq,[0,10],[0,0],method=m)
    print(m+" took: "+str(time.time()-initial)+ " with status: "+str(sol.status))
    ys.append(sol.y[0])
    ts.append(sol.t)


plt.plot(ts[-1],ys[-1])
plt.plot(ts[0],ys[0])
"""


plt.show()
