from dataCollection import signal
import math 
import numpy as np
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
    index=0
    tolerance=0.00001
    while (currentTime<(endTime-tolerance)):
        u=signal(currentTime)
        uL.append(u)
        omega=omegaL[index]
        i=iL[index]
        sgn=sign(omega)

        omegaDot=-K1*omega+K2*i-K6*sgn-K7*math.e**(-K8*abs(omega))*sgn
        iDot=-K4*omega-K3*i+K5*u

        omegaDotL.append(omegaDot)
        iDotL.append(iDot)
        omegaL.append(omega+timeStep*omegaDot)
        iL.append(i+timeStep*iDot)
        currentTime+=timeStep
        index+=1
    omegaL=np.array(omegaL[1:])
    return uL,iL[1:],omegaL,omegaDot


