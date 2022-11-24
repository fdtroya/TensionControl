import serial
import time


class sensor:


    def __init__(self,com,baud):
        self.ser=serial.Serial(com,baud)
    
    def readSensors(self):
        return self.ser.read(size=32)

s=sensor('COM4',57600)
start=time.time()
current=time.time()
c=0
while(current-start)<10:
    s.readSensors()
    c+=1
    current=time.time()
print("###############################")
print(c/10)
#print(s.readSensors())