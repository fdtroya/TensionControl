import random as rd
import multiprocessing
import time
from simulation import runSim
from dataCollection import timeStep
from dataCollection import endTime
import numpy as np
parameters={"mutation":0.1,"populationSize":500,"iterations":25,"searchRanges":[[0,5],[0,6],[0,7],[0,4],[0,3],[0,2],[0,1],[0,3]]}

class Model:   

    def __init__(self,constants):
        self.constants=constants
        self.omegaL=[]
        self.iL=[]
        self.timeSteps=[]
        self.fitness=0
        self.rank=0
        listU,self.iL,self.omegaL,omegaDot=runSim(self.constants,timeStep,endTime)
    


class GA:
    
    u=[]
    
    def __init__(self,parameters,realDataOutput,realDataInput):
        self.parameters=parameters
        self.currentPop=[]
        self.experimentalData=realDataOutput
        global experimentalOmega
        experimentalOmega=self.experimentalData[:,2]
        GA.u=realDataInput
        self.bestIndividual=[]
        self.bestScore=0

        

    def randomIndividual(self,i):
        ind=[]
        for rang in self.parameters["searchRanges"]: 
            ind.append(rd.uniform(rang[0],rang[1]))
        ind=Model(ind)
        return ind
        



    def initialPopPar(self):
        size=self.parameters["populationSize"]
        self.randomIndividual(1)
        pool=multiprocessing.Pool(6)
        it=range(size)
        result=pool.map(self.randomIndividual,it)
        self.currentPop=result
        pool.close()
        

    def initialSeq(self):
        size=self.parameters["populationSize"]
        result=[]
        for i in range(size):
            result.append(self.randomIndividual(i))
        self.currentPop=result

  


    def calcIndFitness(individual):
        squareErrorArray=(individual.omegaL-experimentalOmega)**2
        squareErrorTot=np.sum(squareErrorArray)
        individual.fitness=1/(squareErrorTot+1)
    

    def fitnessCalcSeq(self):
        for individual in self.currentPop:
            GA.calcIndFitness(individual)
"""
    def initWorkerFitnessPool(data):
        global experimentalOmega
        experimentalOmega=data

    
    def fitnessCalcPar(self):
        pool=multiprocessing.Pool(6,initializer=GA.initWorkerFitnessPool,initargs=experimentalOmega)
        pool.map(GA.calcIndFitness,self.currentPop)
"""

    def fitnessRanking(self):
        

if __name__ == '__main__':  
    outputData=np.load("dataConv.npy")
    inputData=np.load("u.npy")  
    alg=GA(parameters,outputData,inputData)
    alg.initialPopPar()

    start=time.time()
    alg.fitnessCalcSeq()
    end=time.time()
    print(end-start)
    print("end")
    




