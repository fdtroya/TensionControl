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
from simple_pid import PID

def sign(n):
    return int(n>0) - int(n<0)

class myThreadloadCell(object):

    def __init__(self,sensorCFG,FtM,freq=100000000):
        self.h=1/freq
        self.FtM=FtM
        self.sensorCFG=sensorCFG


    def run(self,FtM,t):
        sens=sensor(self.sensorCFG[0],self.sensorCFG[1])
        time_after_loop = time.perf_counter()
        while True:
            time_before_loop = time.perf_counter()
            if time_before_loop - time_after_loop >= self.h:
                FtM[0]=sens.readSensors()[0]
                time_after_loop = time.perf_counter()


    def start(self):
        p=Process(target=self.run,args=(self.FtM,0))
        p.start()

class myThreadFrictionEstimator(object):
    def __init__(self,constants,omegaM,TfrM,FtM,h=0.0001):
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
        self.FtM=FtM

    def calcZdot(self,t,z):
        omega=t
        epz=self.sigma0/800
        g=self.Tc+(self.Ts-self.Tc)*math.e**(-((omega/self.Vs)**2))
        g=g/self.sigma0
        zDot=omega-(epz+abs(omega)/g)*z
        return zDot

    def CN(self,guess,omega):
        return -guess+self.z+0.5*self.h*(self.calcZdot(omega,self.z)+self.calcZdot(omega,guess))

    def nextStep(self,omega):
    

        guess=self.z+self.h*self.calcZdot(0,self.z)
        return root(self.CN,guess,args=(omega)).x[0]
        

    def run(self,omegaM,TfrM):
        time_after_loop = time.perf_counter()
        while True:
            time_before_loop = time.perf_counter()
            if time_before_loop - time_after_loop >= self.h:
                omega=self.omegaM[0]
                if(abs(omega)<=0.023981):
                    ft=self.FtM[0]
                    if(abs(ft)>=0.1):
                        s=sign(ft)
                    else:
                        s=0
                    omega=(0.023981/4)*(-s)
                self.z=self.nextStep(omega)
                if(abs(self.z)<=1e-6 and omega==0):
                    self.z=0
                zD=self.calcZdot(0,self.z)
                Tfr=self.sigma0*self.z+self.sigma1*zD
                print(self.z)
                TfrM[0]=Tfr
                time_after_loop = time.perf_counter()

    def start(self):
        p=Process(target=self.run,args=(self.omegaM,self.TfrM))
        p.start()

class myThreadController(object):

    def __init__(self,P,I,FtM,outputTM,setPoint,freq=1000):
        self.P=P
        self.I=I
        self.FtM=FtM
        self.outputTM=outputTM
        self.h=1/freq
        self.setPoint=setPoint

    def run(self,FtM,outputTM):
        controller=PID(self.P,self.I,0,setpoint=self.setPoint)
        controller.sample_time=self.h
        controller.output_limits = (-2.7, 2.7)
        while True:
            Ft=FtM[0]
            output = controller(Ft)
            outputTM[0]=output
    def start(self):
        p=Process(target=self.run,args=(self.FtM,self.outputTM))
        p.start()        


   


class ControlLoop(object):

    def __init__(self,motorController,sensorCFG,frequency,constants,gains,omegaM,TfrM,FtM,outputTM,r=(22.28/2)*10**-3,motorId=1,motordirection=1):

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
        self.freq=frequency
        self.h=1/frequency
        self.constants=constants
        
        self.log=[]
        self.Tfd=1

        self.omegaM=omegaM
        self.TfrM=TfrM
        self.FtM=FtM
        self.outputTM=outputTM

        frConstants=[sigma0,sigma1,sigma2,Ts,Tc,Vs]
        self.r=r

        loadCellthread=myThreadloadCell(sensorCFG,FtM)
        frictionEstimator=myThreadFrictionEstimator(frConstants,omegaM,TfrM,FtM,1e-5)
        self.controller=myThreadController(gains[0],gains[1],FtM,outputTM,self.Tfd,frequency)

        frictionEstimator.start()
        loadCellthread.start()
        
        

    def Tm_d(self,Tfr,Ftd):
        return (Ftd*self.r*0.4)+(Tfr)
    
    def Id(self,Tm):
        return Tm/self.Kt


    def loop(self):
        data=self.motorController.readBulkSensors(self.motorID)*motor.bulkConversion
        self.log.append(data)
        omega=data[2]
        theta=data[3]
        i=data[1]/1000
        self.omegaM[0]=omega
        Tfr=self.TfrM[0]

        T_Forward=self.Tm_d(Tfr,self.Tfd)
        T_FeedBack=0#self.outputTM[0]
        total=T_Forward+T_FeedBack
        
        i_d=self.Id(total)
        pos=theta*(14/360)*5
        if(((pos>=52)and i_d>0) or ((pos<=0)and i_d<0)):
            i_d=0
            print("limits")
        self.motorController.setGoalCurrent(i_d*1000,self.motorID)

    
    def startLoop(self):
        self.motorController.connect([self.motorID])
        self.motorController.homming(self.motorID)
        self.motorController.setCurrentControlMode(self.motorID)
        self.motorController.enableTorque(self.motorID)
        self.controller.start()
        
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
    gains=[0.01,0.05]

    omegaM = Manager().list([0])
    TfrM = Manager().list([0])
    FtM=Manager().list([0])
    outputTM=Manager().list([0])


    sensorCFG=('COM5',38400)
    motors=motor("COM3",4000000)

    freq=1000


    l=ControlLoop(motors,sensorCFG,freq,constants,gains,omegaM,TfrM,FtM,outputTM)
    l.run()