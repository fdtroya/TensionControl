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
import numpy as np
from threading import Thread

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

    def __init__(self,motorController,frequency,constants,gains,ids,setpointsM,omegaM,TfrM,FtM,outputTM,r=(22.28/2)*10**-3,motordirection=1):

        self.constants1=constants[0]
        self.constants2=constants[1]
        self.controllers=[]
        self.gains=gains

        

        frConstants1=constants[0][5:]
        frConstants2=constants[1][5:]
        self.r=r

        self.motorController=motorController
        self.motorIDs=ids
        self.motorDir=motordirection
        self.freq=frequency
        self.h=1/frequency
        
        

        self.omegaM=omegaM
        self.TfrM=TfrM
        self.FtM=FtM
        self.outputTM=outputTM
        self.setpointsM=setpointsM

        controllerForce1=PID(0.4,0.8,0,setpoint=0.05)
        controllerForce1.sample_time=(self.h)
        controllerForce1.output_limits=(0,4.368)


        controllerForce2=PID(0.4,0.8,0,setpoint=0.05)
        controllerForce2.sample_time=(self.h)
        controllerForce2.output_limits=(0,4.368)

        forceControllers=[controllerForce1,controllerForce2]
        self.controllers=forceControllers


        

        frictionEstimator1=myThreadFrictionEstimator(0,frConstants1,omegaM,TfrM,outputTM,FtM,1e-5)
        frictionEstimator2=myThreadFrictionEstimator(1,frConstants2,omegaM,TfrM,outputTM,FtM,1e-5)

        frictionEstimator1.start()
        frictionEstimator2.start()
        
    def stifnessData(self):
        self.motorController.disableTorque(self.motorIDs[0])
        pos=0
        datapoints=[]
        dataForce=0
        while pos<4.2 and dataForce<=16:
            self.motorController.setGoalPositionAngle(pos*(180/3.141516),self.motorIDs[1])
            time.sleep(0.05)
            dataPos=self.motorController.readBulkSensors(self.motorIDs[1])*motor.bulkConversion
            dataPos=dataPos[3]
            dataForce=self.FtM[1]
            datapoints.append([dataPos,dataForce])
            pos-=0.01
        self.motorController.disableTorque(self.motorIDs[1])
        self.motorController.disableTorque(self.motorIDs[0])
        datapoints=np.array(datapoints)
        plt.plot(datapoints[:,0],datapoints[:,1])
        plt.show()
        np.save("stiffnessData2.npy",datapoints)
        exit()       
    
    def Id(self,Tm):
        return Tm/self.Kt
    

    
    def updateSetPoints(self):
        i=0
        for controller in self.controllers:
            controller.setpoint=self.setpointsM[i]
            i+=1
        

    
    def startLoop(self):

        controllerForce1,controllerForce2=self.controllers
        out=0

        controllerForce1.setpoint=0.1
        controllerForce2.setpoint=0.1
        while(out<=0.025):
            force1=self.FtM[0]
            force2=self.FtM[1]
            posSetPoint1=controllerForce1(force1)
            posSetPoint2=controllerForce2(force2)
            self.motorController.setGoalPositionAngle(posSetPoint1*(180/3.141516),self.motorIDs[0])
            self.motorController.setGoalPositionAngle(-posSetPoint2*(180/3.141516),self.motorIDs[1])
            out=max(force1,force2)
        
        controllerForce1.Kp=self.gains[0]
        controllerForce1.Ki=self.gains[1]
        controllerForce1.Kd=self.gains[2]
        

        controllerForce2.Kp=self.gains[0]
        controllerForce2.Ki=self.gains[1]
        controllerForce2.Kd=self.gains[2]
        


        
    def run(self,container1,container2):
        self.startLoop()
        time_after_loop = time.perf_counter()
        beg=time_after_loop
        log=[]
        s=False

        while True:
            time_before_loop = time.perf_counter()
            data=self.loop()
            ti=time_before_loop-beg
            data[0]=ti
            if(container1!=None and container2!=None):
                container1.display(data[2])
                container2.display(data[4])
            #log.append(data)
            #if(ti>=30 and not s):
            #    s=True
            #    print("Saving")
            #    np.save("experimentalResponse.npy",np.array(log))
            #    #exit()
            time_after_loop = time.perf_counter()
            time_to_sleep=self.h-(time_after_loop-time_before_loop)
            if(time_to_sleep>0):
                time.sleep(time_to_sleep)


    


    def loop(self):
        
        forceC1=self.controllers[0]
        forceC2=self.controllers[1]


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



       

        
        
        return [0,theta1,force1,theta2,force2]
        
        

      
class controlWrapper:

    def __init__(self):
        constantsJson=json.load(open("models.json"))
        motors=["motorFr_1","motorFr_2"]
        parameters="Ra,La,J,Kt,B,sigma0,sigma1,sigma2,Ts,Tc,Vs".split(",")
        self.constants=[]



        for m in motors:
            p=[]
            for parameter in parameters:
                p.append(constantsJson[m][parameter])
            self.constants.append(p)


        self.motorPort="COM3"
        self.sensorPort="COM1"

        self.motorIDs=[1,2]
        self.motorController=""

        self.gains=[0.000296,0.0304,8.1*10**-7]#PID constants 0.445,0.445,0

        self.omegaM = Manager().list([0,0])
        self.TfrM = Manager().list([0,0])
        self.FtM=Manager().list([0,0])
        self.outputTM=Manager().list([0,0])
        self.setpointsM=Manager().list([0,0])
        self.freq=900


    

    def connect(self, container1=None,container2=None):
        sensorCFG=(self.sensorPort,38400)
        self.motorController=motor(self.motorPort,4000000)
        self.motorController.connect(self.motorIDs)
        loadCellthread=myThreadloadCell(sensorCFG,self.FtM)
        loadCellthread.start()
        d=1
        for id in self.motorIDs:
            d=d*-1
            self.motorController.homming(id,direction=d)
            self.motorController.setCurrenBasedPositionControlMode(id)
            self.motorController.setCurrentLimit(300,id)
            self.motorController.enableTorque(id)
        
        self.controlLoop=ControlLoop(self.motorController,self.freq,self.constants,self.gains,self.motorIDs,self.setpointsM,self.omegaM,self.TfrM,self.FtM,self.outputTM)
        #p=Process(target=self.controlLoop.run)
        #p.start()
        t=Thread(target=self.controlLoop.run,args=(container1,container2))
        t.start()
        return "Connected"

    def updateSetPoints(self,setpoints):
        self.setpointsM[0]=setpoints[0]
        self.setpointsM[1]=setpoints[1]
        self.controlLoop.updateSetPoints()
        return "SetPoints Updated"
        
        

    def homeMotors(self):
        if (self.motorController==""):
            return "No motor Connected"
        d=1
        for id in self.motorIDs:
            d=d*-1
            self.motorController.homming(id,direction=d)
            self.motorController.setCurrenBasedPositionControlMode(id)
            self.motorController.setCurrentLimit(300,id)
            self.motorController.enableTorque(id)
        return "Done Homming"



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


    gains=[0.000296,0.0104,8.1*10**-7]#0.445,0.445,0

    omegaM = Manager().list([0,0])
    TfrM = Manager().list([0,0])
    FtM=Manager().list([0,0])
    outputTM=Manager().list([0,0])
    setpointsM=Manager().list([1,1])
    


    sensorCFG=('COM6',38400)
    motorController=motor("COM3",4000000)

    ids=[1,2]
    
    freq=900


    l=ControlLoop(motorController,sensorCFG,freq,constants,gains,ids,setpointsM,omegaM,TfrM,FtM,outputTM)
    l.run()