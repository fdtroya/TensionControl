from dataCollection import signal
import math 
import numpy as np
import matplotlib.pyplot as plt



currentTime=0
omegaL=[0]
iL=[0]
omegaDotL=[]
iDotL=[]
uL=[]



"""
B=0.2
J=0.02
Kt=0.015
Ke=0.015
La=0.5
Ra=2
"""
Ra=8.1
Dmotor=13.4*10**-3
La=0.28*10**-3
J=5.41*10**-6
Ke=0.0102469
Kt=10.2*10**-3 
B=3.121*10**-6
timeStep=0.001


def sat(sig,threshold):
    if sig>threshold:
        return threshold
    elif(sig<-threshold):
        return -threshold
    return sig


endTime=10
times=[]
tolerance=0.0001
while (currentTime<=(endTime+tolerance)):
    times.append(currentTime)
    u=signal(currentTime)
    uL.append(u)
    omega=omegaL[-1]
    i=iL[-1]
    

    omegaDot=-(B/J)*omega+(Kt/J)*i
    iDot=-(Ke/La)*omega-(Ra/La)*i+u

    omegaDotL.append(omegaDot)
    iDotL.append(iDot)
    omegaL.append(sat(omega+timeStep*omegaDot,10000))
    iL.append(i+timeStep*iDot)
    currentTime+=timeStep
    
omegaL=np.array(omegaL[1:-1])
plt.plot(times[:-1],omegaL)
plt.plot(times[:-1],uL[:-1])
plt.show()
