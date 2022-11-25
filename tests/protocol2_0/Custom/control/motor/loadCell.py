import serial
import time


class sensor:


    def __init__(self,com,baud):
        ser=serial.Serial()
        ser.port(com)
        ser.baudrate=baud
        ser.bytesize=serial.EIGHTBITS
        ser.parity=serial.PARITY_NONE
        ser.stopbits=serial.STOPBITS_ONE
        try: 
            ser.open()
        except Exception :
            print ("error open serial port: ")
            exit()
        self.ser=ser

    
    def readSensors(self):
        return self.ser.read(size=32)

s=sensor('COM4',38400)
start=time.time()
current=time.time()
c=0
while(current-start)<10:
    print(s.readSensors())
    c+=1
    current=time.time()
print("###############################")
print(c/10)
#print(s.readSensors())