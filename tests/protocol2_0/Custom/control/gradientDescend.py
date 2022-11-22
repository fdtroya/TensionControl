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
    
    

  

    
outputData=np.load("dataConv.npy")
Model.experimentalOmega=outputData[:,2]*353.5
m=Model([53.051396219594615, 39.22563892696018, 45.91351853161375, 19.337626439031325, 22.414007075978255, 0.8403205655849757, 0.9069716705745678, 0.00011904809104546013])

gradient_descent(np.array(m.constants),calcGradient,0.05,1000)


    