import numpy as np
from scipy.optimize import root

class Method(object):
    def __init__(self,derivative:callable,step:float,initial_StateVar:iter,final_t=10):    
        
        self.derivative=derivative
        self.t=[0]
        self.y=[initial_StateVar]
        self.step=step
        self.final_t=final_t
        


    def method(self):
        return 0
    

    def guess(self):
        stateVar_i=self.y[-1]
        t_i=self.t[-1]

        derivatives=self.derivative(t_i,stateVar_i)
        return stateVar_i+self.step*derivatives

    def isSolved(self):
        return self.t[-1]>=self.final_t

    def solve(self):
        while not self.isSolved():
            ti=self.t[-1]

            stateVar_i1=self.method()
            t_i1=ti+self.step

            self.t.append(t_i1)
            self.y.append(stateVar_i1)
        self.y=np.array(self.y)
        self.t=np.array(self.t)
        self.done=True
        

class euler_delante(Method):
    def __init__(self,derivative,step,initial_StateVar,final_t=10):
        super().__init__(derivative,step,initial_StateVar,final_t)
        self.name="Forwards Euler"
    def method(self):

        stateVar_i1=self.guess()

        return stateVar_i1

class euler_atras(Method):

    def __init__(self,derivative,step,initial_StateVar,final_t=10):
        super().__init__(derivative,step,initial_StateVar,final_t)
        self.name="Backwards Euler"

    def function(self,stateVar_i1):
        stateVar_i=self.y[-1]
        t_i=self.t[-1]
        

        return stateVar_i1-stateVar_i-self.step*self.derivative(t_i+self.step,stateVar_i1)

    def method(self):
        guess=self.guess()
        return root(fun=self.function,x0=guess,jac=False).x 

class CrankNicolson(Method):
    def __init__(self,derivative,step,initial_StateVar,final_t=10):
        super().__init__(derivative,step,initial_StateVar,final_t)
        self.name="Crank Nicolson"
        
    def function(self,stateVar_i1):
        stateVar_i=self.y[-1]
        t_i=self.t[-1]


        return stateVar_i1-stateVar_i-(self.step/2)*(self.derivative(t_i,stateVar_i)+self.derivative(t_i+self.step,stateVar_i1))

    def method(self):
        guess=self.guess()
        return root(fun=self.function,x0=guess,jac=False).x

class Heun(Method):
    def __init__(self,derivative,step,initial_StateVar,final_t=10):
        super().__init__(derivative,step,initial_StateVar,final_t,)
        self.name="Heun"
    def method(self):
        stateVar_i=self.y[-1]
        t_i=self.t[-1]
        
        stateVar_i1=stateVar_i+(self.step/2)*(self.derivative(t_i,stateVar_i)+self.derivative(t_i+self.step,self.guess()))
        return stateVar_i1



class PuntoMedio(Method):
    def __init__(self,derivative,step,initial_StateVar,final_t=10):
        super().__init__(derivative,step,initial_StateVar,final_t,)
        self.name="Middle Point"
    def method(self):
        stateVar_i=self.y[-1]
        input_i=self.input[-1]
        t_i=self.t[-1]
        stateVar_i1=stateVar_i+self.step*self.derivative(t_i+0.5*self.step,stateVar_i+0.5*self.step*self.derivative(t_i,stateVar_i))
        return stateVar_i1



class RungeKutta4th(Method):
    def __init__(self,derivative,step,initial_StateVar,final_t=10):
        super().__init__(derivative,step,initial_StateVar,final_t,)
        self.name="Runge Kutta 4th Order"
    def method(self):
        stateVar_i=self.y[-1]
        t_i=self.t[-1]
        input_i=self.input[-1]
        h=self.step
        k1=h*self.derivative(t_i,stateVar_i)
        k2=h*self.derivative(t_i+0.5*h,stateVar_i+0.5*k1)
        k3=h*self.derivative(t_i+0.5*h,stateVar_i+0.5*k2)
        k4=h*self.derivative(t_i+h,stateVar_i+k3)
        stateVar_i1=stateVar_i+k1/6+k2/3+k3/3+k4/6
        return stateVar_i1

class RungeKuttaFehlberg(Method):
    def __init__(self,derivative,step,initial_StateVar,final_t=10):
        super().__init__(derivative,step,initial_StateVar,final_t,)
        self.name="Runge Kutta Fehlberg"