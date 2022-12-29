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
    def __init__(self,constants,omegaM,TfrM,outputTM,FtM,h=0.0001):
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
        self.outputTM=outputTM
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
                omega=omegaM[0]
                if(abs(omega)<0.023):
                    output=self.outputTM[0]-self.FtM[0]*(22.28/2)*10**-3
                    s=sign(output)   
                    if(abs(output)<0.0001):
                        s=0        
                    omega=(0.023981/2)*s
                self.z=self.nextStep(omega)
                if(abs(self.z)<=1e-6 and omega==0):
                    self.z=0
                zD=self.calcZdot(0,self.z)
                Tfr=self.sigma0*self.z+self.sigma1*zD
                TfrM[0]=Tfr
                time_after_loop = time.perf_counter()

    def start(self):
        p=Process(target=self.run,args=(self.omegaM,self.TfrM))
        p.start()

class myThreadController(object):

    def __init__(self,P,I,D,FtM,outputTM,setPoint,freq=10000):
        self.gains=[P,I,D]
        self.FtM=FtM
        self.outputTM=outputTM
        self.h=1/freq
        self.setPoint=setPoint

    def run(self,FtM,outputTM,gains):
        controller=PID(gains[0],gains[1],gains[2],setpoint=self.setPoint)
        controller.sample_time=self.h
        controller.output_limits = (0, 4.6675078387)
        while True:
            Ft=FtM[0]
            output = controller(Ft)
            outputTM[0]=output
    def start(self):
        p=Process(target=self.run,args=(self.FtM,self.outputTM,self.gains))
        p.start()        



class ControlLoop(object):

    def __init__(self,motorController,sensorCFG,frequency,constants,gains,setpoint,omegaM,TfrM,FtM,outputTM,r=(22.28/2)*10**-3,motorId=1,motordirection=1):

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
        self.Tfd=setpoint

        self.omegaM=omegaM
        self.TfrM=TfrM
        self.FtM=FtM
        self.outputTM=outputTM

        frConstants=[sigma0,sigma1,sigma2,Ts,Tc,Vs]
        self.r=r

        loadCellthread=myThreadloadCell(sensorCFG,FtM)
        frictionEstimator=myThreadFrictionEstimator(frConstants,omegaM,TfrM,outputTM,FtM,1e-5)
        self.controller=myThreadController(gains[0],gains[1],gains[2],FtM,outputTM,self.Tfd,frequency)

        frictionEstimator.start()
        loadCellthread.start()
        
        

    def Tm_d(self,Tfr,Ftd):
        return (Ftd*self.r*0)+(Tfr)*0.5
    
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

        T_Forward=self.Tm_d(Tfr,self.Tfd)*0
        T_FeedBack=self.outputTM[0]*360/(2*3.1415)
        total=T_FeedBack+T_Forward
        
        #i_d=self.Id(total)
        pos=theta*(14/360)*5
        #if(((pos>=52)and i_d>0) or ((pos<=0)and i_d<0)):
         #   i_d=0
        print(total)
        self.motorController.setGoalPositionAngle(total,self.motorID)

    
    def startLoop(self):
        self.motorController.connect([self.motorID])
        self.motorController.homming(self.motorID)
        self.motorController.setExtendedPositionControlMode(self.motorID)
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
    gains=[0.36648,4.7603,0.008]#0.03265,0.011597,0.0096#0.191188,0.032766,0.092667

    omegaM = Manager().list([0])
    TfrM = Manager().list([0])
    FtM=Manager().list([0])
    outputTM=Manager().list([0])


    sensorCFG=('COM5',38400)
    motors=motor("COM3",4000000)

    freq=900


    l=ControlLoop(motors,sensorCFG,freq,constants,gains,0.7,omegaM,TfrM,FtM,outputTM)
    l.run()