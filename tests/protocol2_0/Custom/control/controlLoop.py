from motor.motor import motor
from motor.loadCell import sensor
import time
import math
import threading




def sign(n):
    return int(n>0) - int(n<0)

class myThreadloadCell(threading.Thread):

    def __init__(self,sensor,force,freq=60):
        threading.Thread.__init__(self)
        self.sensor=sensor
        self.h=1/freq
        self.force=force
    def run(self):
        while True:
            time_before_loop = time.perf_counter()
            if time_before_loop - time_after_loop >= self.h:
                self.force[0]=self.sensor.readSensors()
                time_after_loop = time.perf_counter()
            


class myThreadFrictionEstimator(threading.Thread):
    def __init__(self,constants,Tfr,Omega,h=0.0001):
        threading.Thread.__init__(self)
        self.Tfr=Tfr
        self.omega=Omega
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

    def loop(self):
        omega=self.omega[0]
        g=self.Tc+(self.Ts-self.Tc)*math.e**(-((omega/self.Vs)**2))
        g=g/self.sigma0
        a=abs(omega)/g
        b=omega
        x=-a*self.h
        self.z=self.z*math.e**(x)+((math.e**(x)-1)/math.log(math.e**a))*b
        self.calcTfr() 
        
    def calcTfr(self):
        omega=self.omega[0]
        g=self.Tc+(self.Ts-self.Tc)*math.e**(-((omega/self.Vs)**2))
        g=g/self.sigma0
        zDot=omega-abs(omega)/g
        Tfr=self.sigma0*self.z+self.sigma1*zDot+self.sigma2*omega
        self.Tfr[0]=Tfr

    def run(self):
        time_after_loop = time.perf_counter()
        while True:
            time_before_loop = time.perf_counter()
            if time_before_loop - time_after_loop >= self.h:
                self.loop()
                time_after_loop = time.perf_counter()


   


class ControlLoop(object):


    def __init__(self,motorController,sensor,frequency,constants,gains):
        self.motorController=motorController
        self.sensor=sensor
        self.freq=frequency
        self.constants=constants
        self.gains=gains
        self.log=[]
        
        self.force=[0]
        self.omega=[0]
        self.Tfr=[0]

        loadCellthread=myThreadloadCell(self.sensor,self.force)
        frictionEstimator=myThreadFrictionEstimator(self.constants,self.Tfr,self.omega,0.0001)
        loadCellthread.start()
        frictionEstimator.start()

    def Tm_d(self,omega,Tfr,Ftd):
        return self.constants[0]*omega+((Ftd*self.constants[5]))+(Tfr)
    
    def Id(self,Tm):
        return Tm/self.constants[1]


    def loop(self):
        data=self.motorController.readBulkSensors()*motor.bulkConversion
        self.log.append(data)
        omega=data[2]
        theta=data[3]
        i=data[1]/1000


    id=1
    def startLoop(self):




        self.motorController.connect(id)
        self.motorController.homming(id)
        self.motorController.setCurrentControlMode(id)
        self.motorController.enableTorque(id)
        
        Ftd=0*9.81
        omega=0
        theta=0
        Tfr=0
        
        while True:
            Tm=self.Tm_d(omega,Tfr,Ftd)
            Id=self.Id(Tm)
            pos=theta*(14/360)*5

            if(((pos>=52)and Id>0) or ((pos<=0)and Id<0)):
                Id=0
                
            self.motorController.setGoalCurrent(Id*1000)

            Tfr=self.Tfr(omega)
            

             #   loopEndtime=time.perf_counter()
        






r1=(22.28/2)*10**-3
Ra=8.1
La=0.28*10**-3

J=5.41*10**-8
Ke=0.0102469
Kt=10.2*10**-3 

B=3.121*10**-7
constants=[B,Kt,Ke,La,Ra,r1]
gains=[]
sens=sensor('COM5',38400)
motors=motor(1,"COM3",4000000)
freq=60


l=loop(motors,sens,freq,constants,gains)

l.startLoop()