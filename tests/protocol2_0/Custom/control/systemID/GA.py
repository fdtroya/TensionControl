import random as rd
import multiprocessing
import time
import numpy as np
import simulations.Model as Model
import math
from operator import attrgetter
from scipy import interpolate
import matplotlib.pyplot as plt
endTime=10
Ra=8.1
La=0.28*10**-3

J=5.41*10**-8
Ke=0.0102469
Kt=10.2*10**-3 
B=0#3.121*10**-7

sigma0=0.00001
sigma1=0.00001
sigma2=0.0001
Tc=0.05
Ts=0.14
Vs=6.106*10**-2

r=(22.28/2)*10**-3
Ra, La, J, Kt, B,sigma0,sigma1,sigma2,Ts,Tc,Vs=[Ra, La, J, Kt, 0.0, 2.822972442765255e-06, 4.179463886982079e-06, 2.425631784317123e-05, 1.6291874078276243, 0.13508707850430987, 5.438225126970719]
parameters={"mutation":0.03,"mutationRange":[0.1,10],"crossover":0.9,"geneTransfer":0.5,"populationSize":100,"stopErr":0.9999,"q":0.25}

def signal(t):# t in seconds [0 10]
    return (3*math.sin((math.pi/endTime)*t),0) 


class GA:
    
   
    
    def __init__(self,parameters,realDataOutput):
        self.parameters=parameters
        self.currentPop=[]
        self.currentPopulationweights=[]
        self.experimentalData=realDataOutput
        global experimentalOmega
        self.experimentalOmegaf=interpolate.interp1d(realDataOutput[:,-1], realDataOutput[:,0],fill_value=0,bounds_error=False)
        self.bestIndividual=None
        self.CurrentmeanFitness=0
        self.bestScore=0

    


    def randomIndividual(self,i):
        l=0.9
        l1=0
        u=1.1
        u1=50
        Rar=rd.uniform(l*Ra,u*Ra)
        Lar=rd.uniform(l*La,u*La)
        Jr=rd.uniform(l*J,u*J)
        Br=rd.uniform(l*B,u*B)
        Ktr=rd.uniform(l*Kt,u*Kt)
        Tcr=rd.uniform(l1*Tc,u*Tc)
        Tsr=rd.uniform(l1*Ts,u*Ts)
        sigma0r=rd.uniform(l1*sigma0,u1*sigma0)
        sigma1r=rd.uniform(l1*sigma1,u1*sigma1)
        sigma2r=rd.uniform(l1*sigma2,u1*sigma2)
        Vsr=rd.uniform(l1*Vs,u1*Vs)
        constants=[Rar, Lar, Jr, Ktr, Br,sigma0r,sigma1r,sigma2r,Tsr,Tcr,Vsr]
        ind=Model.motorFrictionModel(constants=constants,r=r,initialState=[0,0,0],inputFunction=signal)
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
        self.CurrentmeanFitness=0
        for individual in self.currentPop:
            ts=individual.t
            experimentalOmega=np.clip(self.experimentalOmegaf(ts),0,None)
            #plt.plot(ts,individual.y[0])
            #plt.plot(ts,experimentalOmega)
            #plt.plot(self.experimentalData[:,-1], self.experimentalData[:,0])
            #plt.show()
            
            squareErrorArray=(individual.y[0]-experimentalOmega)**2
            squareErrorTot= np.sum(squareErrorArray)
            individual.fitness=1/(squareErrorTot+1)
            self.CurrentmeanFitness+=individual.fitness
        self.currentPop=sorted(self.currentPop,key=attrgetter("fitness"),reverse=True)
        self.CurrentmeanFitness=self.CurrentmeanFitness/len(self.currentPop)

        if(self.bestIndividual==None):
            self.bestIndividual=self.currentPop[0]
        elif(self.bestIndividual.fitness<self.currentPop[1].fitness):
            self.bestIndividual=self.currentPop[1]
        self.bestScore= self.bestIndividual.fitness

    def probabilityCalculation(self):
        q=self.parameters["q"]
        m=self.parameters["populationSize"]
        qPrime=q/(1-((1-q)**m))
        self.currentPopulationweights=[]
        for individual_index in range(len(self.currentPop)):
            individual=self.currentPop[individual_index]
            individual.rank=individual_index+1
            individual.prob=qPrime*((1-q)**(individual.rank-1))
            self.currentPopulationweights.append(individual.prob)
        
    def crossover(self,indv1,indv2):
        child=indv1.constants.copy()
        for gene in range(len(indv1.constants)):
            if(rd.uniform(0,1)<=self.parameters["geneTransfer"]):
                child[gene]=indv2.constants[gene]
            if (rd.uniform(0,1)<=self.parameters["mutation"]):
                child[gene]=child[gene]*rd.uniform(self.parameters["mutationRange"][0],self.parameters["mutationRange"][1])
        return Model.motorFrictionModel(constants=child,r=r,initialState=[0,0,0],inputFunction=signal)
    
    def crossover2(self,indv1,indv2):
        pre=(np.array(indv1.constants)+np.array(indv2.constants))/2
        return Model.motorFrictionModel(constants=list(pre),r=r,initialState=[0,0,0],inputFunction=signal)

    def mutate(self,individual):
        result=individual.constants.copy()
        for i in range(len(individual.constants)):
            if (rd.uniform(0,1)<=self.parameters["mutation"]):
                result[i]=result[i]*rd.uniform(self.parameters["mutationRange"][0],self.parameters["mutationRange"][1])
        return Model.motorFrictionModel(constants=result,r=r,initialState=[0,0,0],inputFunction=signal)

    def nextGenInternal(self,i):

        if(rd.uniform(0,1)<=self.parameters["crossover"]):
            indv1,indv2=rd.choices(self.currentPop,weights=self.currentPopulationweights,k=2)
            return self.crossover2(indv1,indv2)
        else:
            indv1=rd.choices(self.currentPop,weights=self.currentPopulationweights,k=1)[0]
            return self.mutate(indv1)

    def nextGeneration(self):
        nextGen=[self.bestIndividual]
        for i in range(self.parameters["populationSize"]-1):
            nextGen.append(self.nextGenInternal(i))
        self.currentPop=nextGen


    def nextGenerationParallel(self): #does not work
        pool=multiprocessing.Pool()
        nextGen=list(pool.map(self.nextGenInternal,range(self.parameters["populationSize"]-1)))
        nextGen.append(self.bestIndividual)
        self.currentPop=nextGen

        


if __name__ == '__main__':  
    outputData=np.load("dataFrFiltered.npy")#change name 
    outputData[:,0]=outputData[:,0]*353.5

    
    cont=2
    print("Generation 1")
    alg=GA(parameters,outputData)
    alg.initialSeq()
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
        print("Mean Fitness: "+str(alg.CurrentmeanFitness))
        print("Iteration Time "+str(iterationEnd-iterationStart))
        print(alg.bestIndividual.constants)
        cont+=1
    end=time.time()
    print(alg.bestIndividual.constants)
    print("end")
    




