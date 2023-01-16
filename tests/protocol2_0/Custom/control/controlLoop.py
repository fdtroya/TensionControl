from motor.motor import motor
from motor.loadCell import sensor
import time
import math

import json
from multiprocessing import Process
from multiprocessing import Manager
import matplotlib.pyplot as plt
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
        
            FtM[0]=sens.readSensors()[0]
            time_after_loop = time.perf_counter()
            time_to_sleep=self.h-(time_after_loop-time_before_loop)
            if(time_to_sleep>0):
                time.sleep(time_to_sleep)     


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
            time_to_sleep=self.h-(time_after_loop-time_before_loop)
            if(time_to_sleep>0):
                time.sleep(time_to_sleep)                         

    def start(self):
        p=Process(target=self.run,args=(self.omegaM,self.TfrM))
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

        frictionEstimator.start()
        loadCellthread.start()
        
        

    def Tm_d(self,Tfr,Ftd):
        return (Ftd*self.r*0)+(Tfr)
    
    def Id(self,Tm):
        return Tm/self.Kt


    def loop(self,forceController,positionController):
        data=self.motorController.readBulkSensors(self.motorID)*motor.bulkConversion
        self.log.append(data)
        omega=data[2]
        theta=data[3]*(3.141516/180)
        i=data[1]/1000
        self.omegaM[0]=omega
        Tfr=self.TfrM[0]

        posSetPoint=forceController(self.FtM[0])
        print(posSetPoint)

        self.motorController.setGoalPositionAngle(posSetPoint*(180/3.141516),self.motorID)
        """
        positionController.setpoint=posSetPoint

        T_FeedBack=positionController(theta)
        T_Forward=self.Tm_d(Tfr,self.Tfd)

        total=T_FeedBack+T_Forward*0
        self.motorController.setGoalCurrent(total*1000,self.motorID)
        """

        pos=theta*(14/2*3.141516)*5
        force=self.FtM[0]
        return theta,force
        
        

      

    
    def startLoop(self):
        
        self.motorController.connect([self.motorID])
        self.motorController.homming(self.motorID)
        
        #self.motorController.setCurrentControlMode(self.motorID)
        self.motorController.setExtendedPositionControlMode(self.motorID)
        self.motorController.enableTorque(self.motorID)
       
        
    def run(self):
        plt.figure()
        time.sleep(3)
        self.startLoop()
        time_after_loop = time.perf_counter()
        beg=time_after_loop
        t=[]
        thetas=[]
        fs=[]

        controllerForce=PID(0.375,1.76,0.0,setpoint=1.5)
        controllerForce.sample_time=(self.h)
        controllerForce.output_limits=(0,4.368)


        controllerPos=PID(0.168,9.73,0.000728)
        controllerPos.sample_time=self.h
        controllerPos.output_limits = (-1, 1)

        while True:
            time_before_loop = time.perf_counter()
            theta,force=self.loop(controllerForce,controllerPos)
            ti=time_before_loop-beg
            t.append(ti)
            thetas.append(theta)
            fs.append(force)
            plt.plot(t,fs)
            plt.pause(0.005)
            if(ti>=7):
                break
            time_after_loop = time.perf_counter()
            time_to_sleep=self.h-(time_after_loop-time_before_loop)
            if(time_to_sleep>0):
                time.sleep(time_to_sleep)
            
        plt.show()     

    



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
    gains=[0.445,0.445,0]#0.445,0.445,0

    omegaM = Manager().list([0])
    TfrM = Manager().list([0])
    FtM=Manager().list([0])
    outputTM=Manager().list([0])


    sensorCFG=('COM6',38400)
    motors=motor("COM3",4000000)

    freq=900


    l=ControlLoop(motors,sensorCFG,freq,constants,gains,1.575,omegaM,TfrM,FtM,outputTM)
    l.run()