from motor.motor import motor
from motor.loadCell import sensor
import time
import math
import threading
import json
from multiprocessing import Process
from multiprocessing import Manager
import scipy.integrate as scI
from scipy.optimize import root

def sign(n):
    return int(n>0) - int(n<0)

class myThreadloadCell(object):

    def __init__(self,sensor,force,freq=60):
        #threading.Thread.__init__(self)
        self.sensor=sensor
        self.h=1/freq
        self.force=force
    def run(self):
        time_after_loop = time.perf_counter()
        while True:
            time_before_loop = time.perf_counter()
            if time_before_loop - time_after_loop >= self.h:
                self.force[0]=self.sensor.readSensors()[0]
                time_after_loop = time.perf_counter()
    def start(self):
        p=Process(target=self.run)
        p.start()
            


class myThreadFrictionEstimator(object):



    def __init__(self,constants,omegaM,TfrM,h=0.0001):
        #threading.Thread.__init__(self)

        self.z=0
        self.h=h
        sigma0,sigma1,sigma2,Ts,Tc,Vs=constants
        self.sigma0=sigma0
        self.sigma1=sigma1
        self.sigma2=sigma2
        self.Ts=Ts
        self.Tc=Tc
        self.Vs=Vs
        self.frequency=1/h
        self.omegaM=omegaM
        self.TfrM=TfrM
        

    def calcZdot(self,t,z):
        omega=self.omegaM[0]
        g=self.Tc+(self.Ts-self.Tc)*math.e**(-((omega/self.Vs)**2))
        g=g/self.sigma0
        zDot=omega-(abs(omega)/g)*z
        return zDot

    def CN(self,guess):
        return -guess+self.z+0.5*self.h*(self.calcZdot(0,self.z)+self.calcZdot(0,guess))

    def nextStep(self,omega):
        if(omega!=0):
            g=self.Tc+(self.Ts-self.Tc)*math.e**(-((omega/self.Vs)**2))
            g=g/self.sigma0
            a=abs(omega)/g
            b=omega
            x=-a*self.h
            z=(math.e**x)*self.z+((math.e**x)-1)/(math.log(math.e**a))*b
            return z 
        else:
            return self.z

        

    def run(self,omegaM,TfrM):
        time_after_loop = time.perf_counter()
        while True:
            time_before_loop = time.perf_counter()
            if time_before_loop - time_after_loop >= self.h:
                omega=omegaM[0]
                self.z=self.nextStep(omega)
                Tfr=self.sigma0*self.z+self.sigma1*self.calcZdot(0,self.z)+self.sigma2*omega
                #print(time_before_loop - time_after_loop)
                TfrM[0]=Tfr
                time_after_loop = time.perf_counter()

    def start(self):
        p=Process(target=self.run,args=(self.omegaM,self.TfrM))
        p.start()


   


class ControlLoop(object):


    def __init__(self,motorController,sensor,frequency,constants,omegaM,TfrM,FtM,r=(22.28/2)*10**-3,motorId=1,motordirection=1):

        Ra, La, J, Kt, B,sigma0,sigma1,sigma2,Ts,Tc,Vs=constants

        self.Kt=Kt
        self.B=B
        self.sigma0=sigma0
        self.sigma1=sigma1
        self.sigma2=sigma2
        self.Ts=Ts
        self.Tc=Tc
        self.Vs=Vs 

        self.motorController=motorController
        self.motorID=motorId
        self.motorDir=motordirection
        self.sensor=sensor
        self.freq=frequency
        self.h=1/frequency
        self.constants=constants
        
        self.log=[]
        self.Tfd=0

        self.omegaM=omegaM
        self.TfrM=TfrM
        self.FtM=FtM

        frConstants=[sigma0,sigma1,sigma2,Ts,Tc,Vs]
        self.r=r
        #loadCellthread=myThreadloadCell(self.sensor,self.force)
        self.frictionEstimator=myThreadFrictionEstimator(frConstants,omegaM,TfrM,1e-5)
        self.frictionEstimator.start()
        #loadCellthread.start()
        

    def Tm_d(self,Tfr,Ftd):
        return (Ftd*self.r)+(Tfr)
    
    def Id(self,Tm):
        return Tm/self.Kt


    def loop(self):
        data=self.motorController.readBulkSensors(self.motorID)*motor.bulkConversion
        self.log.append(data)
        omega=data[2]
        theta=data[3]
        i=data[1]/1000
        self.omegaM[0]=omega

        Tfr=0
        f=self.TfrM[0]
        if(f>1):
            print(f)
        
        i_d=self.Id(self.Tm_d(Tfr,self.Tfd))
        


        pos=theta*(14/360)*5
        
        
        if(((pos>=52)and i_d>0) or ((pos<=0)and i_d<0)):
            i_d=0

        if(abs(Tfr)<=1 or i_d==0):
            self.motorController.setGoalCurrent(i_d*1000,self.motorID)

    
    def startLoop(self):
        self.motorController.connect([self.motorID])
        self.motorController.homming(self.motorID)
        self.motorController.setCurrentControlMode(self.motorID)
        self.motorController.enableTorque(self.motorID)
        
    def run(self):
        self.startLoop()
        time_after_loop = time.perf_counter()
        while True:
            time_before_loop = time.perf_counter()
            if time_before_loop - time_after_loop >= self.h:
                self.loop()
                time_after_loop = time.perf_counter()

    



if __name__ == '__main__':

    constants=json.load(open("models.json"))
    Ra=constants["motorFr_1"]["Ra"]
    La=constants["motorFr_1"]["La"]
    J=constants["motorFr_1"]["J"]
    Kt=constants["motorFr_1"]["Kt"]
    B=constants["motorFr_1"]["B"]
    sigma0=constants["motorFr_1"]["sigma0"]
    sigma1=constants["motorFr_1"]["sigma1"]
    sigma2=constants["motorFr_1"]["sigma2"]
    Ts=constants["motorFr_1"]["Ts"]
    Tc=constants["motorFr_1"]["Tc"]
    Vs=constants["motorFr_1"]["Vs"]
    constants=[Ra, La, J, Kt, B,sigma0,sigma1,sigma2,Ts,Tc,Vs]

    omegaM = Manager().list([0])
    TfrM = Manager().list([0])
    FtM=Manager().list([0])


    sens=5#sensor('COM5',38400)


    motors=motor("COM3",4000000)
    freq=1000


    l=ControlLoop(motors,sens,freq,constants,omegaM,TfrM,FtM)
    l.run()