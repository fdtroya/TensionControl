import numpy as np
import simulations.Model as Model
import math
from scipy import interpolate
from scipy.optimize import minimize
outputData=np.load("dataFrFiltered.npy")#change name 
outputData[:,0]=outputData[:,0]*353.5
Ra=8.1
La=0.28*10**-3

J=5.41*10**-8
Ke=0.0102469
Kt=10.2*10**-3 
B=3.121*10**-7
r=(22.28/2)*10**-3
endTime=10
experimentalOmegaf=interpolate.interp1d(outputData[:,-1], outputData[:,0],fill_value=0,bounds_error=False)



def signal(t):# t in seconds [0 10]
    return (3*math.sin((math.pi/endTime)*t),0) 


def func(x):
    sigma0,sigma1,sigma2,Ts,Tc,Vs=list(x)
    constants=[Ra, La, J, Kt, B,sigma0,sigma1,sigma2,Ts,Tc,Vs]
    individual=Model.motorFrictionModel(constants=constants,r=r,initialState=[0,0,0],inputFunction=signal)
    ts=individual.t
    experimentalOmega=np.clip(experimentalOmegaf(ts),0,None)
    squareErrorArray=(individual.y[0]-experimentalOmega)**2
    squareErrorTot= np.sum(squareErrorArray)
    return 1/(squareErrorTot+1)

s=minimize(func,[4.45788784e-02 ,-2.73756718e-03 , 6.44971886e-03 , 9.73832395e+13 ,3.62708135e-02 , 8.45934619e+06])
print(s.x)
print(s.success)
print(s.message)