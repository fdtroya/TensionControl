import random as rd
import multiprocessing
import time
from simulation import runSim
from dataCollection import timeStep
from dataCollection import endTime
import numpy as np
from operator import attrgetter


parameters={"mutation":0.1,"mutationRange":[0.8,1.2],"crossover":0.9,"geneTransfer":0.5,"populationSize":100,"stopErr":0.9,"searchRanges":[[0,5],[0,6],[0,7],[0,4],[0,3],[0,2],[0,1],[0,3]],"q":0.1}

class Model:   

    def __init__(self,constants):
        self.constants=constants
        self.omegaL=[]
        self.iL=[]
        self.timeSteps=[]
        self.fitness=0
        self.rank=0
        self.prob=0
        listU,self.iL,self.omegaL,omegaDot=runSim(self.constants,timeStep,endTime)
    


class GA:
    
    u=[]
    
    def __init__(self,parameters,realDataOutput,realDataInput):
        self.parameters=parameters
        self.currentPop=[]
        self.currentPopulationweights=[]
        self.experimentalData=realDataOutput
        global experimentalOmega
        experimentalOmega=self.experimentalData[:,2]
        GA.u=realDataInput
        self.bestIndividual=None
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

    


    def fitnessCalcSeq(self):
        for individual in self.currentPop:
            squareErrorArray=(individual.omegaL-experimentalOmega)**2
            squareErrorTot=np.sum(squareErrorArray)
            individual.fitness=1/(squareErrorTot+1)
        self.currentPop=sorted(self.currentPop,key=attrgetter("fitness"),reverse=True)

        if(self.bestIndividual==None):
            self.bestIndividual=self.currentPop[0]
        elif(self.bestIndividual.fitness<self.currentPop[1].fitness):
            self.bestIndividual=self.currentPop[1]
        self.bestScore= self.bestIndividual.fitness

    def probabilityCalculation(self):
        q=self.parameters["q"]
        m=self.parameters["populationSize"]
        qPrime=q/(1-(1-q)**m)
        self.currentPopulationweights=[]
        for individual_index in range(len(self.currentPop)):
            individual=self.currentPop[individual_index]
            individual.rank=individual_index
            individual.prob=qPrime*(1-q)**(individual.rank-1)
            self.currentPopulationweights.append(individual.prob)
        
    def crossover(self,indv1,indv2):
        child=indv1.constants.copy()
        for gene in range(len(indv1.constants)):
            if(rd.uniform(0,1)<=self.parameters["geneTransfer"]):
                child[gene]=indv2.constants[gene]
            if (rd.uniform(0,1)<=self.parameters["mutation"]):
                child[gene]=child[gene]*rd.uniform(self.parameters["mutationRange"][0],self.parameters["mutationRange"][1])
        return Model(child)
    
    def mutate(self,individual):
        result=individual.constants.copy()
        for i in range(len(individual.constants)):
            if (rd.uniform(0,1)<=self.parameters["mutation"]):
                result[i]=result[i]*rd.uniform(self.parameters["mutationRange"][0],self.parameters["mutationRange"][1])
        return Model(result)

    def nextGenInternal(self,i):
        if(rd.uniform(0,1)<=self.parameters["crossover"]):
            indv1,indv2=rd.choices(self.currentPop,weights=self.currentPopulationweights,k=2)
            return self.crossover(indv1,indv2)
        else:
            indv1=rd.choices(self.currentPop,weights=self.currentPopulationweights,k=1)[0]
            return self.mutate(indv1)

    def nextGeneration(self):
        nextGen=[self.bestIndividual]
        for i in range(self.parameters["populationSize"]-1):
            nextGen.append(self.nextGenInternal(i))
        self.currentPop=nextGen
    def nextGenerationParallel(self):
        pool=multiprocessing.Pool()
        nextGen=list(pool.map(self.nextGenInternal,range(self.parameters["populationSize"]-1)))
        nextGen.append(self.bestIndividual)
        self.currentPop=nextGen

        


if __name__ == '__main__':  
    outputData=np.load("dataConv.npy")
    inputData=np.load("u.npy")
    start=time.time()  
    cont=2
    print("Generation 1")
    alg=GA(parameters,outputData,inputData)
    alg.initialPopPar()
    alg.fitnessCalcSeq()
    alg.probabilityCalculation()
    print("Best Fitness: "+str(alg.bestScore))
    while(alg.bestIndividual.fitness<=alg.parameters["stopErr"]):
        iterationStart=time.time()
        print("###################################")
        print("Generation "+str(cont))
        alg.nextGeneration()
        alg.fitnessCalcSeq()
        alg.probabilityCalculation()
        iterationEnd=time.time()
        print("Best Fitness: "+str(alg.bestScore))
        print("Iteration Time "+str(iterationEnd-iterationStart))
        cont+=1
    end=time.time()
  
    print("end")
    



