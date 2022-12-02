import matplotlib.pyplot as plt
import math
class method(object):
 
   
   
    array_real_y=[]
    done=False
    def __init__(self,derivative,real_y,step,initial_p,final_x=None,final_y=None):
        self.initial_p=initial_p
        self.final_x=final_x
        self.step=step
        self.array_ys=[]
        self.derivative=derivative
        self.array_xs=[]
        self.error_abs=[]
        self.error_rel=[]
        x,y=self.initial_p
        self.array_xs.append(x)
        self.array_ys.append(y)
        self.final_y=final_y
        self.name="generic"
        self.averageError=0
        self.real_y=real_y
        if(len(self.array_real_y)==0):
            self.array_real_y.append(y)
     
        
        
    def method(self):
        
        return 1
    

    def eval_derivative(self,x,y):
        return self.derivative(x,y)
      
    
    def doMethod(self):
        def test():
            if (self.final_y==None):
                return self.array_xs[-1]<=self.final_x
            elif (self.eval_derivative(self.array_xs[-1],self.array_ys[-1])<=0):
                return self.array_ys[-1]>=self.final_y
            else:
                return self.array_ys[-1]<self.final_y

        while test():
            xi=self.array_xs[-1]

            x_i1=xi+self.step
            y_i1=self.method()
            
            self.array_xs.append(x_i1)
            self.array_ys.append(y_i1)
            if(not (self.done)):
                self.array_real_y.append(self.real_y(x_i1))
        print(x_i1)
        method.done=True
        
    def plot(self):
        if (len(self.array_xs)==1):
            self.doMethod()
        plt.plot(self.array_xs,self.array_ys,'b-',label="Aproximacion")
        plt.plot(self.array_xs,self.array_real_y,'r-',label="Solucion Analitica")
        plt.legend(loc='upper right')
        plt.title(self.name)
        plt.show()
