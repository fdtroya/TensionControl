import numpy as np
import math
class Model(object):


    def __init__(self,stateEq,inputFunction,initialState):
        self.stateEq=stateEq
        self.stateVarLog=[]
        self.inputLog=[]
        self.tLog=[]
        self.inputFunction=inputFunction
        self.initialState=initialState
    

    def evalStateEq(self,t,stateVector):
        stateVarDot=[]
        input=self.inputFunction(t)
        for eq in self.stateEq:
            stateVarDot.append(eq(stateVector,input))
        return np.array(stateVarDot)


class motorModel(Model):

    def omegaDot(self,stateVar,input):
        omega,i=stateVar
        v=input[0]
        omegaDotJ=-self.B*omega+i*self.Kt
        return omegaDotJ/self.J

    def iDot(self,stateVar,input):
        v,Ft=input
        omega,i=stateVar
        iDotL=-self.Ke*omega-self.Ra*i+v
        return iDotL/self.La

    def __init__(self, Ra,La,J,Ke,Kt,B,inputFunction,initialState):
        self.Ra=Ra
        self.La=La
        self.J=J
        self.Ke=Ke
        self.Kt=Kt
        self.B=B
        stateEq=[self.omegaDot,self.iDot]
        super().__init__(stateEq,inputFunction,initialState)
    
    

class motorFrictionModel(motorModel):

    def calcStribertEffect(self,omega):
        g=self.Tc+(self.Ts-self.Tc)*math.e**-((omega/self.Vs)**2)
        return g/self.sigma0

    def calcZDot(self,stateVar,input):
        omega,i,z=stateVar
        g=self.calcStribertEffect(omega)
        zDot=omega-(abs(omega)/g)*z
        return zDot

    def omegaDot(self, stateVar, input):
        omega,i,z=stateVar
        v,Ft=input
        zDot=self.calcZDot(stateVar,input)
        Tfr=self.sigma0*z+self.sigma1*zDot+self.sigma2*omega
        omegaDotJ=-self.B*omega+i*self.Kt-Ft*self.r-Tfr
        return omegaDotJ/self.J
        
    def iDot(self,stateVar,input):
        v,Ft=input
        omega,i,z=stateVar
        iDotL=-self.Ke*omega-self.Ra*i+v
        return iDotL/self.La
        

    


    def __init__(self, Ra, La, J, Ke, Kt, B,sigma0,sigma1,sigma2,Ts,Tc,Vs,r,inputFunction,initialState):
        super().__init__(Ra, La, J, Ke, Kt, B, inputFunction,initialState)
        self.stateEq=[self.omegaDot,self.iDot,self.calcZDot]
        self.sigma0=sigma0
        self.sigma1=sigma1
        self.sigma2=sigma2
        self.Ts=Ts
        self.Tc=Tc
        self.Vs=Vs
        self.r=r

        self.calculated=False
        self.used=False
        self.zDot=0
        

