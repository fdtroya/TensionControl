import random as rd
import multiprocessing
import time
from simulation import runSim
parameters={"mutation":0.1,"populationSize":200,"iterations":25,"searchRanges":[[0,5],[0,6],[0,7],[0,4],[0,3],[0,2],[0,1]]}

class Model:   

    def __init__(self,constants):
        self.constants=constants
        self.omegaL=[]
        self.iL=[]
        self.timeSteps=[]
        self.sim()
    def sim(self):
        listU,self.iL,self.omegaL,omegaDot=runSim(self.constants,GA.u,GA.timeStep,GA.endtime)
        return True


class GA:
    timeStep=0.001
    endTime=10
    u=[]

    def __init__(self,parameters,realData,timeStep):
        self.parameters=parameters
        self.currentPop=[]
        self.experimentalData=realData
        self.timeStep=timeStep
        self.bestIndividual=[]
        self.bestScore=0

    def randomIndividual(self,i):
        ind=[i]
        for rang in self.parameters["searchRanges"]: 
            ind.append(rd.uniform(rang[0],rang[1]))
        return ind
    def initialPop2(self):
        size=self.parameters["populationSize"]
        pool=multiprocessing.Pool(6)
        result=pool.map(self.randomIndividual,range(size))
        return result

    def initialPop(self):
        size=self.parameters["populationSize"]
        result=[]
        for i in range(size):
            result.append(self.randomIndividual(i))
        return result
    def sim(individual):
        return individual.sim()
    def fitnessCalcind(self):


start=time.time()
if __name__ == '__main__':    
    alg=GA(parameters)
    r=alg.initialPop()
    end=time.time()
    print(end-start)
    print(r)




