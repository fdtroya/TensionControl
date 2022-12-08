import numpy as np
import simulations.Model as Model
import math
from scipy import interpolate
from scipy.optimize import minimize
from scipy.optimize import differential_evolution
from scipy.optimize import basinhopping
from scipy.optimize import shgo
from scipy.optimize import  dual_annealing
import matplotlib.pyplot as plt

outputDataFr=np.load("dataFrFiltered.npy")#change name 
outputData=np.load("dataHFiltered.npy")#change name 
Ra=8.1
La=0.28*10**-3

J=(5.41*353.5)*10**-8
Kt=2.67857142857
B=(3*353.5)*10**-7
r=(22.28/2)*10**-3
min=10**-6



bounds=[(8,9.5),(0.26*10**-3,0.32*10**-3),(9*10**-4,7*10**-3),(2.65,2.8),(9*10**-2,7*10**-1)]
guess=[9.25170038e+00 ,2.74492995e-04, 6.94938581e-03 ,2.68651742e+00,2.89755777e-01]

bounds1=[(9*10**-4,7*10**-3),(2.65,2.8),(9*10**-2,7*10**-1)]
guess1=[2.16827014e-03, 2.74042159e+00 ,2.95628771e-01]

boundsFr=[(85000,100000),(2500,4500),(0.05,0.07),(0.15,0.3),(0.02,0.17),(0.0000001,0.001)]
guessFr=[8.79424207e+04, 3.28712928e+03, 6.71277751e-02 ,1.58548656e-01,1.50433937e-01, 6.18313380e-04]

endTimeFr=10
endTime=10
Ra,La,J,Kt,B=guess


experimentalOmegaFrf=interpolate.interp1d(outputDataFr[:,-1], outputDataFr[:,0],fill_value=0,bounds_error=False)
experimentalOmegaf=interpolate.interp1d(outputData[:,-1], outputData[:,0],fill_value=0,bounds_error=False)


def signalFr(t):# t in seconds [0 10]
    return (3*math.sin((math.pi/endTimeFr)*t),0) 


def signal(t):# t in seconds [0 10]
    return (7*math.sin((math.pi/endTime)*t),0) 

def func(x,plot=False):
    Ra,La,J,Kt,B=list(x)
    
    individual=Model.motorModel(Ra,La,J,Kt,B,initialState=[0,0],inputFunction=signal)
    f=interpolate.interp1d(individual.t, individual.y[0],fill_value=0,bounds_error=False)
    ts=np.arange(0,endTime,0.001)
    experimentalOmega=np.clip(experimentalOmegaf(ts),0,None)
    individualOmega=f(ts)
    squareErrorArray=(individualOmega-experimentalOmega)**2
    if(plot):
        plt.plot(individual.t,individual.y[0])
        plt.plot(ts,experimentalOmega)
        plt.show()
    squareErrorTot= np.sum(squareErrorArray)
    return squareErrorTot

def evolve():
    strats='best1exp'
    maxIter=300
    popSize=100
    mutation=(0.5,1.5)
    CR=0.25
    disp=True
    polish=True
    init='halton'
    atol=0.0001
    s=differential_evolution(func,strategy=strats,maxiter=maxIter,popsize=popSize,mutation=mutation,recombination=CR,disp=disp,polish=polish,bounds=bounds,init=init,atol=atol,updating='immediate',x0=guess)
    print(s.x)
    print(s.success)
    print(s.message)
    print(func(s.x,plot=True))
def Hopping():
    maxIter=1000
    s=basinhopping(func,guess)
    print(s.x)
    print(s.success)
    print(s.message)
    print(func(s.x,plot=True))
def SH():
    maxIter=5
    s=shgo(func,bounds=bounds,iters=maxIter,options={'disp':True})
    print(s.x)
    print(s.success)
    print(s.message)
    print(func(s.x,plot=True))  
def dual():
    maxIter=1000
    s=dual_annealing(func,bounds=bounds,maxiter=maxIter,x0=guess)
    print(s.x)
    print(s.success)
    print(s.message)
    print(func(s.x,plot=True))  

def funcFr(x,plot=False):
    interest=10
    sigma0,sigma1,sigma2,Ts,Tc,Vs=list(x)
    constants=[Ra, La, J, Kt, B,sigma0,sigma1,sigma2,Ts,Tc,Vs]
    individual=Model.motorFrictionModel(constants=constants,r=r,initialState=[0,0,0],inputFunction=signalFr,timeRange=[0,interest])
    f=interpolate.interp1d(individual.t, individual.y[0],fill_value=0,bounds_error=False)
    ts=np.arange(0,interest,0.001)
    experimentalOmega=np.clip(experimentalOmegaFrf(ts),0,None)
    individualOmega=f(ts)
    squareErrorArray=(individualOmega-experimentalOmega)**2
    if(plot):
        plt.plot(individual.t,individual.y[1])
        plt.plot(ts,experimentalOmega)
        plt.show()
    squareErrorTot= np.sum(squareErrorArray)
    return squareErrorTot
def minimization():
    methods=['Nelder-Mead','Powell','L-BFGS-B','TNC','COBYLA','trust-constr']
    for method in methods:
        s=minimize(func,guess1,method=method,bounds=bounds1)
        print(method)
        print(s.x)
        print(s.success)
        print(s.message)
        print(func(s.x,True))
        print("################################")


def minimizationFr():

    methods=['Nelder-Mead','Powell','L-BFGS-B','TNC','COBYLA','trust-constr']
    for method in methods:
        s=minimize(funcFr,guessFr,method=method,bounds=boundsFr)
        print(method)
        print(s.x)
        print(s.success)
        print(s.message)
        print(funcFr(s.x))
        print("################################")

def evolveFr():
    strats='best1bin'
    maxIter=50
    popSize=100
    mutation=(0.2,1.5)
    CR=0.8
    disp=True
    polish=True
    init='sobol'
    atol=0.0001
    s=differential_evolution(funcFr,strategy=strats,maxiter=maxIter,popsize=popSize,mutation=mutation,recombination=CR,disp=disp,polish=polish,bounds=boundsFr,init=init,atol=atol,updating='immediate',x0=guessFr)
    print(s.x)
    print(s.success)
    print(s.message)
    print(funcFr(s.x,True))
def SHFr():
    maxIter=4
    s=shgo(funcFr,bounds=boundsFr,iters=maxIter,options={'disp':True})
    print(s.x)
    print(s.success)
    print(s.message)
    print(funcFr(s.x,True))  
def dualFr():
    maxIter=1000
    s=dual_annealing(funcFr,bounds=boundsFr,maxiter=maxIter,x0=guessFr)
    print(s.x)
    print(s.success)
    print(s.message)
    print(funcFr(s.x,True)) 
def HoppingFr():
    maxIter=1000
    s=basinhopping(funcFr,guessFr)
    print(s.x)
    print(s.success)
    print(s.message)
    print(funcFr(s.x))

if __name__ == '__main__':  
    #evolve()
    #evolveFr()
    #print(func(guess,True))
    #dualFr()
    #SHFr()
    funcFr(guessFr,True)
    #SH()
    #minimizationFr()