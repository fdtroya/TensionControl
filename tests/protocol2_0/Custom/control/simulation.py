from dataCollection import signal
import math 
import numpy as np
import matplotlib.pyplot as plt
"""
start=time.time()
for i in range(10000):
    a=math.sign(i)
end=time.time()
"""

def sign(n):
    return int(n>0) - int(n<0)
def runSim(listK,timeStep,endTime):
    currentTime=0
    omegaL=[0]
    iL=[0]
    omegaDotL=[]
    iDotL=[]
    uL=[]
    K1,K2,K3,K4,K5,K6,K7,K8=listK
    times=[]
    tolerance=0.0001
    while (currentTime<=(endTime+tolerance)):
        times.append(currentTime)
        u=signal(currentTime)
        uL.append(u)
        omega=omegaL[-1]
        i=iL[-1]
        sgn=sign(omega)

        omegaDot=-K1*omega+K2*i-K6*sgn-K7*math.e**(-K8*abs(omega))*sgn
        iDot=-K4*omega-K3*i+K5*u

        omegaDotL.append(omegaDot)
        iDotL.append(iDot)
        omegaL.append(omega+timeStep*omegaDot)
        iL.append(i+timeStep*iDot)
        currentTime+=timeStep
        
    omegaL=np.array(omegaL[1:-1])
    #plt.plot(times[:-1],omegaL)
    #plt.show()
    return uL,iL[1:],omegaL,omegaDot


