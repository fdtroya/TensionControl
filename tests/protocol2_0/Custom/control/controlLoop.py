from motor.motor import motor
from motor.loadCell import sensor
import time
import threading




def sign(n):
    return int(n>0) - int(n<0)

class myThreadloadCell(threading.Thread):

    def __init__(self,sensor,start_event,end_event,force):
        threading.Thread.__init__(self)
        self.sensor=sensor
        self.start_event = start_event
        self.end_event = end_event
        self.force=force
    def run(self):
        while True:
            self.start_event.wait()
            self.start_event.clear()
            self.force[0]=self.sensor.readSensors()
            self.end_event.set()




class loop(object):


    def __init__(self,motorController,sensor,frequency,constants,gains):
        self.motorController=motorController
        self.sensor=sensor
        self.freq=frequency
        self.constants=constants
        self.gains=gains
        self.log=[]

    def Tm(self,omega,Tfr,Ftd):
        return self.constants[0]*omega*353.5+((Ftd*self.constants[5])/353.5)+(Tfr)
    
    def Id(self,Tm):
        return Tm/self.constants[1]
    
    def Tfr(self,omega,Ft,i):
        return i*self.constants[1]-(self.constants[0]*omega*353.5)-((Ft*self.constants[5])/353.5)-(0.1*9.81*self.constants[5]/353.5)*sign(Ft)



    def startLoop(self):

        loadCellStartEvent=threading.Event()
        loadCellEndEvent=threading.Event()
        force=[0]
        loadCellthread=myThreadloadCell(self.sensor,loadCellStartEvent,loadCellEndEvent,force)
        loadCellthread.start()
        loadCellStartEvent.set()

        self.motorController.connect()
        self.motorController.setCurrentControlMode()
        self.motorController.setHome()
        self.motorController.enableTorque()
        
        Ftd=0*9.81
        omega=0
        theta=0
        Tfr=0
        
        while True:

            Tm=self.Tm(omega,Tfr,Ftd)
            Id=self.Id(Tm)
            pos=theta*(14/360)*5
            if(((pos>=52)and Id>0) or ((pos<=0)and Id<0)):
                Id=0
                
            
            
            print("Corriente " +str(Id))
            print("pos "+str(pos))
                
            self.motorController.setGoalCurrent(Id*1000)

            data=self.motorController.readBulkSensors()*motor.bulkConversion
            self.log.append(data)
            omega=data[2]
            theta=data[3]

            i=data[1]/1000


            loadCellEndEvent.wait()
            loadCellEndEvent.clear()

            Ft=force[0][0]*9.81

            

            loadCellStartEvent.set()

            Tfr=self.Tfr(omega,Ft,i)
            

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