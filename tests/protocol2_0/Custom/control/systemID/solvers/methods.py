import Method
from Solver import secant_method
class euler_delante(Method.method):
    def __init__(self,derivative,real_y,step,initial_p,final_x=None,final_y=None):
        super().__init__(derivative,real_y,step,initial_p,final_x,final_y)
        self.name="Euler hacia Delante"
    def method(self):
        y=self.array_ys[-1]
        x=self.array_xs[-1]
        yi_1=y+self.step*self.eval_derivative(x,y)
        return yi_1

class euler_atras(Method.method):
    def __init__(self,derivative,real_y,step,initial_p,final_x=None,final_y=None):
        super().__init__(derivative,real_y,step,initial_p,final_x,final_y)
        self.name="Euler hacia Atras"
    def function(self,yi_1):
        return yi_1-self.array_ys[-1]-self.step*self.eval_derivative(self.array_xs[-1]+self.step,yi_1)
    def method(self):
        guess=self.array_ys[-1]+self.step*self.eval_derivative(self.array_xs[-1],self.array_ys[-1])
        return secant_method(self.function,guess)

class CrankNicolson(Method.method):
    def __init__(self,derivative,real_y,step,initial_p,final_x=None,final_y=None):
        super().__init__(derivative,real_y,step,initial_p,final_x,final_y)
        self.name="Crank Nicolson"
    def function(self,yi_1):
        return yi_1-self.array_ys[-1]-(self.step/2)*(self.eval_derivative(self.array_xs[-1],self.array_ys[-1])+self.eval_derivative(self.array_xs[-1]+self.step,yi_1))
    def method(self):
        guess=self.array_ys[-1]+self.step*self.eval_derivative(self.array_xs[-1],self.array_ys[-1])
        return secant_method(self.function,guess)

class Heun(Method.method):
    def __init__(self,derivative,real_y,step,initial_p,final_x=None,final_y=None):
        super().__init__(derivative,real_y,step,initial_p,final_x,final_y)
        self.name="Heun"
    def method(self):
        y=self.array_ys[-1]
        x=self.array_xs[-1]
        yi_1=y+(self.step/2)*(self.eval_derivative(x,y)+self.eval_derivative(x+self.step,y+self.step*self.eval_derivative(x,y)))
        return yi_1
class PuntoMedio(Method.method):
    def __init__(self,derivative,real_y,step,initial_p,final_x=None,final_y=None):
        super().__init__(derivative,real_y,step,initial_p,final_x,final_y)
        self.name="Punto Medio"
    def method(self):
        y=self.array_ys[-1]
        x=self.array_xs[-1]
        yi_1=y+self.step*self.eval_derivative(x+0.5*self.step,y+0.5*self.step*self.eval_derivative(x,y))
        return yi_1
class RungeKutta4th(Method.method):
    def __init__(self,derivative,real_y,step,initial_p,final_x=None,final_y=None):
        super().__init__(derivative,real_y,step,initial_p,final_x,final_y)
        self.name="Runge Kutta 4to Orden"
    def method(self):
        yi=self.array_ys[-1]
        xi=self.array_xs[-1]
        h=self.step
        k1=self.eval_derivative(xi,yi)
        k2=self.eval_derivative(xi+0.5*h,yi+0.5*h*k1)
        k3=self.eval_derivative(xi+0.5*h,yi+0.5*h*k2)
        k4=self.eval_derivative(xi+h,yi+h*k3)
        yi_1=yi+(h/6)*(k1+2*k2+2*k3+k4)
        return yi_1