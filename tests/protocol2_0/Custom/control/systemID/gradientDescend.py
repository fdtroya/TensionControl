import random as rd
import multiprocessing
import time
from simulation import runSim
from dataCollection import timeStep
from dataCollection import endTime
import numpy as np
from operator import attrgetter
class Model:   

    experimentalOmega=np.array([])
    def __init__(self,constants):
        self.constants=constants
        self.omegaL=[]
        self.iL=[]
        self.timeSteps=[]
        self.error=0
        listU,self.iL,self.omegaL,omegaDot=runSim(self.constants,timeStep,endTime)
        self.caclError()

    def caclError(self):
        squareErrorArray=(self.omegaL-Model.experimentalOmega)**2
        squareErrorTot=np.sum(squareErrorArray)
        self.error=squareErrorTot
        

def calcGradient(x):
    ep=0.001
    h=ep*x
    xPh=x+h
    xMh=x-h
    
    m1=Model(list(xPh))
    m2=Model(list(xMh))
    print(m1.error)
    gradient=(m1.error-m2.error)/(2*h)
    return gradient


def gradient_descent(start, gradient, learn_rate, max_iter, tol=0.01):
    steps = [start] # history tracking
    x = start

    for _ in range(max_iter):
        diff = learn_rate*gradient(x)
        if abs(np.linalg.norm(diff)) <tol:
            
            break    
        x = x - diff
        steps.append(x) # history tracing
    
    

  

Ra,La,J,B,Ke,Kt,Tc,Ts,alpha=[8.1,0.28*10**-3,5.41*10**-8,3.121*10**-7,0.0102469,10.2*10**-3,0.048,0.0536,0.002]
c=[B/J,Kt/J,Ra/La,Ke/La,1/La,Tc/J,(Ts-Tc)/J,alpha]
outputData=np.load("dataConv.npy")
Model.experimentalOmega=outputData[:,2]

gradient_descent(np.array(c),calcGradient,0.001,1000)


    