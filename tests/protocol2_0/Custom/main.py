from motor import *
import numpy as np
import time


#position control


dyn1=motor(1,'COM3',4000000)
dyn1.connect()
dyn1.setPWMControlMode()
# Enable Dynamixel Torque 
dyn1.setHome()
dyn1.enableTorque()
dyn1.setGoalPWM(50) 
pos=0
allData=[]
timestart=time.time()
while abs(pos)<720:
    data=dyn1.readBulkSensors()
    allData.append(data)
    pos=data[3]
    pos=motor.numberToAngle(pos)
endTime=time.time()
dyn1.setGoalPWM(0)
dyn1.disableTorque()
data=np.array(allData)
np.save("data.npy",data)
print("ex Time")
t=endTime-timestart
print(t)   
datapoint= np.shape(data)[0]
print("freq=")
print(datapoint/t)
print(datapoint)
