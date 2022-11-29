import serial
import time


class sensor:


    def __init__(self,com,baud):
        ser=serial.Serial(port=com,baudrate=baud,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)

  
        self.ser=ser

    
    def readSensors(self):
        final=[1,0]
        res=str(self.ser.read(size=32))
        result=res.split("S")
        result.reverse()
        result.pop()
        for element in result:
            number=int(element[0])-1
            pred=element[5:13]
            data=float(pred)
            if(abs(data)<=0.002):
                data=0
            final[number]=data

        return final


if __name__ == '__main__':  
    s=sensor('COM4',38400)
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