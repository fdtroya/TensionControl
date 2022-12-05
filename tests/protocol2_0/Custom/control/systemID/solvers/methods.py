import numpy as np
from scipy.optimize import root

class Method(object):
    def __init__(self,inputFunction,step,initial_StateVar,final_t=10):    
        
        self.inputFunction=inputFunction
        self.ts=[0]
        self.stateVarLog=[initial_StateVar]
        self.input=[self.inputFunction(self.ts[-1])]
        self.step=step
        self.final_t=final_t
        
    def bindStateVarDerivative(self,stateVarDerivative):
        self.stateVarDerivative=stateVarDerivative

    def method(self):
        return 0
    

    def guess(self):
        stateVar_i=self.stateVarLog[-1]
        input_i=self.input[-1]

        derivatives=self.stateVarDerivative(stateVar_i,input_i)
        return stateVar_i+self.step*derivatives

    def isSolved(self):
        return self.ts[-1]>=self.final_t

    def solve(self):
        while not self.isSolved():
            ti=self.ts[-1]

            stateVar_i1=self.method()
            t_i1=ti+self.step

            self.input.append(self.inputFunction(t_i1))
            self.ts.append(t_i1)
            self.stateVarLog.append(stateVar_i1)

        self.done=True

class euler_delante(Method):
    def __init__(self,inputFunction,step,initial_StateVar,final_t=10):
        super().__init__(inputFunction,step,initial_StateVar,final_t)
        self.name="Euler hacia Delante"
    def method(self):

        stateVar_i1=self.guess()

        return stateVar_i1

class euler_atras(Method):

    def __init__(self,inputFunction,step,initial_StateVar,final_t=10):
        super().__init__(inputFunction,step,initial_StateVar,final_t)
        self.name="Euler hacia Atras"

    def function(self,stateVar_i1):
        stateVar_i=self.stateVarLog[-1]
        t_i=self.ts[-1]
        input_i1=self.inputFunction(t_i+self.step)

        return stateVar_i1-stateVar_i-self.step*self.stateVarDerivative(stateVar_i1,input_i1)

    def method(self):
        guess=self.guess()
        return root(fun=self.function,x0=guess,jac=False) 

class CrankNicolson(Method):
    def __init__(self,inputFunction,step,initial_StateVar,final_t=10):
        super().__init__(inputFunction,step,initial_StateVar,final_t)
        self.name="Crank Nicolson"
        
    def function(self,stateVar_i1):
        stateVar_i=self.stateVarLog[-1]
        t_i=self.ts[-1]
        input_i=self.input[-1]
        input_i1=self.inputFunction(t_i+self.step)

        return stateVar_i1-stateVar_i-(self.step/2)*(self.stateVarDerivative(stateVar_i,input_i)+self.eval_derivative(stateVar_i1,input_i1))

    def method(self):
        guess=self.guess()
        return root(fun=self.function,x0=guess,jac=False)

class Heun(Method):
    def __init__(self,inputFunction,step,initial_StateVar,final_t=10):
        super().__init__(inputFunction,step,initial_StateVar,final_t,)
        self.name="Heun"
    def method(self):
        stateVar_i=self.stateVarLog[-1]
        input_i=self.input[-1]
        t_i=self.ts[-1]
        input_i1=self.inputFunction(t_i+self.step)
        stateVar_i1=stateVar_i+(self.step/2)*(self.stateVarDerivative(stateVar_i,input_i)+self.stateVarDerivative(self.guess(),input_i1))
        return stateVar_i1



class PuntoMedio(Method):
    def __init__(self,inputFunction,step,initial_StateVar,final_t=10):
        super().__init__(inputFunction,step,initial_StateVar,final_t,)
        self.name="Punto Medio"
    def method(self):
        stateVar_i=self.stateVarLog[-1]
        input_i=self.input[-1]
        t_i=self.ts[-1]
        stateVar_i1=stateVar_i+self.step*self.stateVarDerivative(stateVar_i+0.5*self.step*self.stateVarDerivative(stateVar_i,input_i),self.inputFunction(t_i+0.5*self.step))
        return stateVar_i1



class RungeKutta4th(Method):
    def __init__(self,inputFunction,step,initial_StateVar,final_t=10):
        super().__init__(inputFunction,step,initial_StateVar,final_t,)
        self.name="Runge Kutta 4to Orden"
    def method(self):
        stateVar_i=self.stateVarLog[-1]
        t_i=self.ts[-1]
        input_i=self.input[-1]
        h=self.step
        k1=h*self.stateVarDerivative(stateVar_i,input_i)
        k2=h*self.stateVarDerivative(stateVar_i+0.5*k1,self.inputFunction(t_i+0.5*h))
        k3=h*self.stateVarDerivative(stateVar_i+0.5*k2,self.inputFunction(t_i+0.5*h))
        k4=h*self.stateVarDerivative(stateVar_i+k3,self.inputFunction(t_i+h))
        stateVar_i1=stateVar_i+k1/6+k2/3+k3/3+k4/6
        return stateVar_i1