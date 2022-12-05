import numpy as np
import math
class Model(object):


    def __init__(self,stateEq,solver):
        self.stateEq=stateEq
        self.solver=solver
        self.stateVarLog=[]
        self.inputLog=[]
        self.tLog=[]
        solver.bindStateVarDerivative(self.evalStateEq)
    

    def evalStateEq(self,stateVar,input):
        stateVarDot=[]
        for eq in self.stateEq:
            stateVarDot.append(eq(stateVar,input))
        return np.array(stateVarDot)
    
    def sim(self):
        self.solver.solve()
        self.stateVarLog=self.solver.stateVarLog
        self.inputLog=self.solver.input
        self.tLog=self.solver.ts
        


class motorModel(Model):

    def omegaDot(self,stateVar,input):
        omega,i=stateVar
        v=input[0]
        omegaDotJ=-self.B*omega+i*self.Kt
        return omegaDotJ/self.J

    def iDot(self,stateVar,input):
        v=input[0]
        omega,i=stateVar
        iDotL=-self.Ke*omega-self.Ra*i+v[0]
        return iDotL/self.La

    def __init__(self, Ra,La,J,Ke,Kt,B,solver):
        self.Ra=Ra
        self.La=La
        self.J=J
        self.Ke=Ke
        self.Kt=Kt
        self.B=B
        stateEq=[self.omegaDot,self.iDot]
        super().__init__(stateEq,solver)
    
    

class motorFrictionModel(motorModel):

    def calcStribertEffect(self,omega):
        g=self.Tc+(self.Ts-self.Tc)*math.e**-((omega/self.Vs)**2)
        return g

    def calcZDot(self,omega,z):
        g=self.calcStribertEffect(omega)
        epzDot=omega-(abs(omega)/g)*z
        return epzDot*self.sigma0

    def omegaDot(self, stateVar, input):
        omega,i,z=stateVar
        v,Ft=input
        z
        omegaDotJ=-self.B*omega+i*self.Kt-Ft*self.r-(z+self.sigma1/)
        

    


    def __init__(self, Ra, La, J, Ke, Kt, B,sigma0,sigma1,sigma2,Ts,Tc,Vs,r,solver):
        super().__init__(Ra, La, J, Ke, Kt, B, solver)
        self.sigma0=sigma0
        self.sigma1=sigma1
        self.sigma2=sigma2
        self.Ts=Ts
        self.Tc=Tc
        self.Vs=Vs
        self.r=r

