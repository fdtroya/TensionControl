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



bounds=[(8,9.5),(0.26*10**-3,0.32*10**-3),(10**-5,2.5*10**-3),(2.65,2.7),(10**-3,6*10**-1)]
bounds1=[(10**-4,4*10**-3),(2.65,2.7),(10**-2,7*10**-1)]

boundsFr=[(0.01,0.06),(0.0005,0.06),(0.0005,0.015),(0.6,1.2),(0.2,0.8),(0.4,0.8)]

guess=[8.71427011e+00 ,2.69348163e-04 ,1.30802543e-03 ,2.66204421e+00,3.94582734e-01]
guess1=[1.54556096e-03 ,2.66925653e+00 ,4.03202592e-01]
guessFr=[0.02548752 ,0.01175936, 0.00247281, 1.10968817 ,0.39299388 ,0.47506953]
endTime=10

Ra,La,J,Kt,B=guess


experimentalOmegaFrf=interpolate.interp1d(outputDataFr[:,-1], outputDataFr[:,0],fill_value=0,bounds_error=False)
experimentalOmegaf=interpolate.interp1d(outputData[:,-1], outputData[:,0],fill_value=0,bounds_error=False)


def signal(t):# t in seconds [0 10]
    return (3*math.sin((math.pi/endTime)*t),0) 

def func(x,plot=False):
    Ra,La,J,Kt,B=list(x)
    
    individual=Model.motorModel(Ra,La,J,Kt,B,initialState=[0,0],inputFunction=signal)
    ts=individual.t
    experimentalOmega=np.clip(experimentalOmegaf(ts),0,None)
    squareErrorArray=(individual.y[0]-experimentalOmega)**2
    #plt.plot(ts,individual.y[0])
    #plt.plot(ts,experimentalOmega)
    #plt.show()
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
    print(func(s.x))
def Hopping():
    maxIter=1000
    s=basinhopping(func,guess)
    print(s.x)
    print(s.success)
    print(s.message)
    print(func(s.x))
def SH():
    maxIter=5
    s=shgo(func,bounds=bounds,iters=maxIter,options={'disp':True})
    print(s.x)
    print(s.success)
    print(s.message)
    print(func(s.x))  
def dual():
    maxIter=1000
    s=dual_annealing(func,bounds=bounds,maxiter=maxIter,x0=guess)
    print(s.x)
    print(s.success)
    print(s.message)
    print(func(s.x))  

def funcFr(x):
    sigma0,sigma1,sigma2,Ts,Tc,Vs=list(x)
    constants=[Ra, La, J, Kt, B,sigma0,sigma1,sigma2,Ts,Tc,Vs]
    individual=Model.motorFrictionModel(constants=constants,r=r,initialState=[0,0,0],inputFunction=signal)
    ts=individual.t
    experimentalOmega=np.clip(experimentalOmegaFrf(ts),0,None)
    squareErrorArray=(individual.y[0]-experimentalOmega)**2
    plt.plot(ts,individual.y[0])
    plt.plot(ts,experimentalOmega)
    plt.show()
    squareErrorTot= np.sum(squareErrorArray)
    return squareErrorTot

def minimizationFr():

    methods=['Nelder-Mead','Powell','L-BFGS-B','TNC','COBYLA','trust-constr',]
    for method in methods:
        s=minimize(funcFr,guessFr,method=method,bounds=bounds)
        print(method)
        print(s.x)
        print(s.success)
        print(s.message)
        print(funcFr(s.x))
        print("################################")

def evolveFr():
    strats='best1exp'
    maxIter=300
    popSize=100
    mutation=(0.5,1.5)
    CR=0.25
    disp=True
    polish=True
    init='halton'
    atol=0.0001
    s=differential_evolution(funcFr,strategy=strats,maxiter=maxIter,popsize=popSize,mutation=mutation,recombination=CR,disp=disp,polish=polish,bounds=boundsFr,init=init,atol=atol,updating='immediate',x0=guessFr)
    print(s.x)
    print(s.success)
    print(s.message)
    print(funcFr(s.x))
def SHFr():
    maxIter=4
    s=shgo(funcFr,bounds=boundsFr,iters=maxIter,options={'disp':True})
    print(s.x)
    print(s.success)
    print(s.message)
    print(funcFr(s.x))  
def dualFr():
    maxIter=1000
    s=dual_annealing(funcFr,bounds=boundsFr,maxiter=maxIter,x0=guessFr)
    print(s.x)
    print(s.success)
    print(s.message)
    print(funcFr(s.x)) 
def HoppingFr():
    maxIter=1000
    s=basinhopping(funcFr,guessFr)
    print(s.x)
    print(s.success)
    print(s.message)
    print(funcFr(s.x))
#evolve()
#evolveFr()
#func(guess)
#dualFr()
#SHFr()
funcFr(guessFr)