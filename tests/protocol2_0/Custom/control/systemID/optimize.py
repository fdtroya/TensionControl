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
import json

def loadJson(file):
    with open(file) as json_file:
        data = json.load(json_file)
        return data

outputDataFr=np.load("dataFrFiltered.npy")#change name 
outputData=np.load("dataHFiltered.npy")#change name 
outputDataStable=np.load("stableStateData.npy")
models=loadJson("models.json")

r=(22.28/2)*10**-3






boundsFr=[(9000,15000),(70,500),(0.03,0.2),(0.11,0.18),(0.03,0.11),(0.002,0.1)]
guessFr=[9288.98500623 ,  97.33555491,1.54433544e-01,1.13292172e-01,1.09904280e-01,0.03]#9.36241349e-02]

boundsFr1=[(9000,15000),(70,500)]
guessFr1=[9288.98500623 ,  97.33555491]


boundsSS=[(0.03,0.1),(0.1,0.18),(0.03,0.12),(0.008,0.1)]
guessSS=[1.54433544e-01,0.14357413,0.10497904,0.03391913]

#sigma2,Ts,Tc,Vs=guessSS

endTimeFr=10
endTime=10
Ra=models["motor"]["Ra"]
La=models["motor"]["La"]
J=models["motor"]["J"]
Kt=models["motor"]["Kt"]
B=models["motor"]["B"]


experimentalOmegaFrf=interpolate.interp1d(outputDataFr[:,-1], outputDataFr[:,0],fill_value=0,bounds_error=False)
experimentalOmegaf=interpolate.interp1d(outputData[:,-1], outputData[:,0],fill_value=0,bounds_error=False)

def sign(n):
    return int(n>0) - int(n<0)


def signalFr(t):# t in seconds [0 10]
    return (3*math.sin((math.pi/endTimeFr)*t),0) 


def signal(t):# t in seconds [0 10]
    return (7*math.sin((math.pi/endTime)*t),0) 


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
        plt.plot(individual.t,individual.y[0])
        plt.plot(ts,experimentalOmega)
        plt.show()
    squareErrorTot= np.sum(squareErrorArray)
    return squareErrorTot
def funcSt(x,plot=False):
    sigma2,Ts,Tc,Vs=x
    interval=[0,3]

    omegaLs=outputDataStable[:,1]
    mask1=omegaLs<=interval[-1]
    mask2=omegaLs>=interval[0]
    mask=mask1*mask2
    omegaLs=omegaLs[mask]
    Torques=outputDataStable[:,0][mask]
    def Tf(omegas):
        TfrLs=[]
        for omega in omegas:
            Tfr=(Tc+(Ts-Tc)*math.e**-(omega/Vs)**2) *sign(omega)+sigma2*omega
            TfrLs.append(Tfr)
        TfrArr=np.array(TfrLs)
        return TfrArr
    err=(Tf(omegaLs)-Torques)**2
    if plot:
        ws=np.arange(-3,3,0.01)
        plt.plot(ws,Tf(ws),'.')
        plt.plot(outputDataStable[:,1],outputDataStable[:,0],'.')
        plt.show()
    return np.sum(err)
    


def evolveFr():
    strats='best1bin'
    maxIter=50
    popSize=50
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
    funcFr(s.x,True)
    return s.x
    
def evolveFrSS():
    strats='best1bin'
    maxIter=250
    popSize=1000
    mutation=(0.2,1.5)
    CR=0.8
    disp=True
    polish=True
    init='sobol'
    atol=0.0000001
    tol=0.00000001
    s=differential_evolution(funcSt,strategy=strats,maxiter=maxIter,popsize=popSize,mutation=mutation,recombination=CR,disp=disp,polish=polish,bounds=boundsSS,init=init,atol=atol,tol=tol,updating='immediate',x0=guessSS)
    print(s.x)
    print(s.success)
    print(s.message)
    return s.x



def SHFr():
    maxIter=4
    s=shgo(funcFr,bounds=boundsFr,iters=maxIter,options={'disp':True})
    print(s.x)
    print(s.success)
    print(s.message)
    print(funcFr(s.x,True)) 
def SHFrSS():
    maxIter=5
    s=shgo(funcSt,bounds=boundsSS,iters=maxIter,options={'disp':True})
    print(s.x)
    print(s.success)
    print(s.message)
    print(funcSt(s.x,True)) 







if __name__ == '__main__':  
    #print(funcFr(guessFr,True))
    #print(funcSt(guessSS,True))
    evolveFr()
