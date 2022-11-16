import numpy as np 
import math 
import time
"""
start=time.time()
for i in range(10000):
    a=math.sign(i)
end=time.time()
"""

def sign(n):
    return (n>0) - (n<0)
def runSim(listK,listU,timeStep,endTime):
    currentTime=0
    omegaL=[0]
    iL=[0]
    omegaDotL=[]
    iDotL=[]
    K1,K2,K3,K4,K5,K6,K7,K8=listK
    index=0
    while (currentTime<=endTime):
        u=listU[index]
        omega=omegaL[index]
        i=iL[index]
        sgn=sign(omega)
        omegaDot=-K1*omega+K2*i-K6*sgn-K7*math.e**(-K8*abs(omega))*sgn
        iDot=-K4*omega-K3*i+K5*u
        omegaDotL.append(omegaDot)
        iDotL.append(iDot)
        omegaL.append(omega+omega*omegaDot)
        iL.append(i+i*iDot)
        currentTime+=timeStep
        index+=1
    return [listU,iL,omega,omegaDot]



#1.0001659393310547