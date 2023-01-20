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
        
            res=sens.readSensors()
            FtM[0]=res[0]
            FtM[1]=res[1]
            time_after_loop = time.perf_counter()
            time_to_sleep=self.h-(time_after_loop-time_before_loop)
            if(time_to_sleep>0):
                time.sleep(time_to_sleep)     


    def start(self):
        p=Process(target=self.run,args=(self.FtM,0))
        p.start()

class myThreadFrictionEstimator(object):
    def __init__(self,num,constants,omegaM,TfrM,outputTM,FtM,h=0.0001):
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
        self.num=num

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
            omega=omegaM[self.num]
            if(abs(omega)<0.023):
                output=self.outputTM[self.num]-self.FtM[self.num]*(22.28/2)*10**-3
                s=sign(output)   
                if(abs(output)<0.0001):
                    s=0        
                omega=(0.023981/2)*s
            self.z=self.nextStep(omega)
            if(abs(self.z)<=1e-6 and omega==0):
                self.z=0
            zD=self.calcZdot(0,self.z)
            Tfr=self.sigma0*self.z+self.sigma1*zD
            TfrM[self.num]=Tfr
            time_after_loop = time.perf_counter()
            time_to_sleep=self.h-(time_after_loop-time_before_loop)
            if(time_to_sleep>0):
                time.sleep(time_to_sleep)                         

    def start(self):
        p=Process(target=self.run,args=(self.omegaM,self.TfrM))
        p.start()




class ControlLoop(object):

    def __init__(self,motorController,sensorCFG,frequency,constants,gains,ids,setpoints,omegaM,TfrM,FtM,outputTM,r=(22.28/2)*10**-3,motorId=1,motordirection=1):

        self.constants1=constants[0]
        self.constants2=constants[1]

        

        frConstants1=constants[0][5:]
        frConstants2=constants[1][5:]
        self.r=r

        self.motorController=motorController
        self.motorIDs=ids
        self.motorDir=motordirection
        self.freq=frequency
        self.h=1/frequency
        
        self.setpoints=setpoints

        self.omegaM=omegaM
        self.TfrM=TfrM
        self.FtM=FtM
        self.outputTM=outputTM

        
        self.overCurrent=0

        loadCellthread=myThreadloadCell(sensorCFG,FtM)

        frictionEstimator1=myThreadFrictionEstimator(0,frConstants1,omegaM,TfrM,outputTM,FtM,1e-5)
        frictionEstimator2=myThreadFrictionEstimator(1,frConstants2,omegaM,TfrM,outputTM,FtM,1e-5)

        frictionEstimator1.start()
        frictionEstimator2.start()
        loadCellthread.start()
        
    
    def Id(self,Tm):
        return Tm/self.Kt
    
    def startLoop(self):
        self.motorController.connect(self.motorIDs)
        d=1
        for id in self.motorIDs:
            d=d*-1
            self.motorController.homming(id,direction=d)
            self.motorController.setCurrenBasedPositionControlMode(id)
            self.motorController.setCurrentLimit(700,id)
            self.motorController.enableTorque(id)
    
       
        
    def run(self):
        self.startLoop()
        time_after_loop = time.perf_counter()
        beg=time_after_loop
        log=[]
       
        controllerForce1=PID(0.02,0.047,0,setpoint=self.setpoints[0])
        controllerForce1.sample_time=(self.h)
        controllerForce1.output_limits=(0,4.368)


        controllerForce2=PID(0.02,0.047,0,setpoint=self.setpoints[1])
        controllerForce2.sample_time=(self.h)
        controllerForce2.output_limits=(0,4.368)


        forceControllers=[controllerForce1,controllerForce2]


        controllerPos=PID(0.168,9.73,0.000728)
        controllerPos.sample_time=self.h
        controllerPos.output_limits = (-1, 1)
        #self.motorController.disableTorque(self.motorIDs[0])
        while True:
            time_before_loop = time.perf_counter()
            data=self.loop(forceControllers,controllerPos)
            ti=time_before_loop-beg
            data[0]=ti
            log.append(data)
            #if(ti>=7):
            #    break
            time_after_loop = time.perf_counter()
            time_to_sleep=self.h-(time_after_loop-time_before_loop)
            if(time_to_sleep>0):
                time.sleep(time_to_sleep)


    


    def loop(self,forceControllers,positionControllers):
        
        forceC1=forceControllers[0]
        forceC2=forceControllers[1]


        data1=self.motorController.readBulkSensors(self.motorIDs[0])*motor.bulkConversion
        data2=self.motorController.readBulkSensors(self.motorIDs[1])*motor.bulkConversion
        
        i1=data1[1]
        theta1=data1[3]
        omega1=data1[2]
        self.omegaM[0]=omega1
        Tfr1=self.TfrM[0]
        force1=self.FtM[0]


        i2=data2[1]
        theta2=data2[3]
        omega2=data2[2]
        self.omegaM[1]=omega2
        Tfr2=self.TfrM[1]
        force2=self.FtM[1]


        posSetPoint1=forceC1(force1)
        posSetPoint2=forceC2(force2)
        

        self.motorController.setGoalPositionAngle(posSetPoint1*(180/3.141516),self.motorIDs[0])
        self.motorController.setGoalPositionAngle(-posSetPoint2*(180/3.141516),self.motorIDs[1])



       

        
        
        return [0,[theta1,force1],[theta2,force2]]
        
        

      




if __name__ == '__main__':

    constantsJson=json.load(open("models.json"))
    motors=["motorFr_1","motorFr_2"]
    parameters="Ra,La,J,Kt,B,sigma0,sigma1,sigma2,Ts,Tc,Vs".split(",")
    constants=[]
    for m in motors:
        p=[]
        for parameter in parameters:
            p.append(constantsJson[m][parameter])
        constants.append(p)


    gains=[0.445,0.445,0]#0.445,0.445,0

    omegaM = Manager().list([0,0])
    TfrM = Manager().list([0,0])
    FtM=Manager().list([0,0])
    outputTM=Manager().list([0,0])
    


    sensorCFG=('COM6',38400)
    motorController=motor("COM3",4000000)

    ids=[1,2]
    setpoints=[3.5,4]
    freq=900


    l=ControlLoop(motorController,sensorCFG,freq,constants,gains,ids,setpoints,omegaM,TfrM,FtM,outputTM)
    l.run()