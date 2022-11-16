
from math import degrees, radians
import keyboard
from threading import Thread
from threading import Lock
import time



from dynamixel_sdk import *                   
from motor.config import *
from motor.addressTable import *
from motor.commandTable import *
import numpy as np

class motor(object):

    #utilities
    pulleyR=11#in mm
    displacementPerRotation=14*5 #in mm
    bulkConversion=np.array([0.112994,2.69,0.0239808239224,0.0879121])
    def toSigned32(n):
        n = n & 0xffffffff
        return (n ^ 0x80000000) - 0x80000000

    def mAmpsToNumber(mAmps): #0-1743.12
        return round(mAmps/2.69)
    def numberTomAmps(number):
        return number*2.69
    def angleToLinear(angle):
        return angle*(motor.displacementPerRotation/360)
    def linearToAngle(linear):
        return 360*(linear/motor.displacementPerRotation)
    def numberToAngle(number):
        return number*0.0879121
    def angleToNumber(angle):
        return round(angle/0.0879121)
    
    def numberToDutyCicle(number):
        return number*(100/885)
    def dutyCycleToNumber(dutyCycle):
        return round(dutyCycle*(885/100))
    def numberToradS(number):
        return number*0.0239808239224
    def radSToNumber(radS):
        return round(radS/0.0239808239224)
    
    def twos_comp(val, bytes):
        bits=8*bytes
        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val 


    def __init__(self,ID,port,BAUDRATE) -> None:
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.BAUDRATE= BAUDRATE
        self.DXL_ID=ID
        self.comLock=Lock()
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)
        
      
        
    def connect(self):

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            raise Exception("Failed to open the port")

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
            keyboard.on_release(lambda e: self.keyBinds(e))
            self.disableTorque()
            self.setHome()
           
        else:
            raise Exception("Failed to change the baudrate")
        
        

    def disConnect(self):
         # Disable Dynamixel Torque
        self.disableTorque()
        # Close port
        self.portHandler.closePort()
        
        

    def writeAddressInner(self,address,value,byteSize):
        self.comLock.acquire()
        if(byteSize==1):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, address, value)
        elif(byteSize==2):
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, address, value)
        elif(byteSize==4):
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, address, value)
        else:
            raise Exception("Invalid byteSize")

        if dxl_comm_result != COMM_SUCCESS:
            raise Exception("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            raise Exception("%s" % self.packetHandler.getRxPacketError(dxl_error))
        
        self.comLock.release()
        return True
    def readAddressInner(self,address,byteSize):
        self.comLock.acquire()
        if(byteSize==1):
            dxl_value, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, address)
        elif(byteSize==2):
            dxl_value, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, address)
        elif(byteSize==4):
            dxl_value, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, address)
        else:
            raise Exception("Invalid byteSize")
        
        
     
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            raise Exception("%s" % self.packetHandler.getRxPacketError(dxl_error))
        self.comLock.release()
        final=motor.twos_comp(dxl_value,byteSize)
        return final

   

    def readBulkSensors(self):
        start=124
        end=132
        l=4
        size=end-start+l
        data,result,error=self.packetHandler.readTxRx(self.portHandler,self.DXL_ID,start,size)
        if (result == COMM_SUCCESS):
            PWMB=DXL_MAKEWORD(data[0], data[1])
            currentB=DXL_MAKEWORD(data[2], data[3])
            velocityB=DXL_MAKEDWORD(DXL_MAKEWORD(data[4], data[5]),
                                  DXL_MAKEWORD(data[6], data[7]))
            positionB=DXL_MAKEDWORD(DXL_MAKEWORD(data[8], data[9]),
                                  DXL_MAKEWORD(data[10], data[11]))

            PWM=motor.twos_comp(PWMB,2)
            current=motor.twos_comp(currentB,2)
            velocity=motor.twos_comp(velocityB,4)
            position=motor.twos_comp(positionB,4)
        else:
            PWM,current,velocity,position=[0,0,0,0]
        return (PWM,current,velocity,position)
        
    def readAddress(self,address,byteSize):
        count=0
        tries=0
        while True:
            try:    
                return self.readAddressInner(address,byteSize)
                
            except Exception as e:
                print(str(e))
                if count>=tries:
                    raise(e)
                time.sleep(0.05)
                count+=1

    def writeAddress(self,address,value,byteSize):
        count=0
        tries=0
        while True:
            try:    
                self.writeAddressInner(address,value,byteSize)
                return
            except Exception as e:
                print(str(e))  
                if count>=tries:
                    raise(e)
                time.sleep(0.05)
                count+=1
    
            
    def enableTorque(self):
        if(not self.isArmed()):
            self.writeAddress(ADDR_PRO_TORQUE_ENABLE,CMD_TORQUE_ENABLE,LEN_PRO_TORQUE_ENABLE)
         

    def disableTorque(self):
        if(self.isArmed()):
            self.writeAddress(ADDR_PRO_TORQUE_ENABLE,CMD_TORQUE_DISABLE,LEN_PRO_TORQUE_ENABLE) 
            
    #Control Modes
    def setControlMode(self,CMD_MODE):
        
        self.writeAddress(ADDR_OPERATING_MODE,CMD_MODE,LEN_OPERATING_MODE)
        

    def setCurrentControlMode(self):
        self.setControlMode(CMD_CURRENT_CONTROL_MODE)
    def setPositionControlMode(self):
        self.setControlMode(CMD_POSITION_CONTROL_MODE)
    def setVelocityControlMode(self):
        self.setControlMode(CMD_VELOCITY_CONTROL_MODE)
    def setPWMControlMode(self):
        self.setControlMode(CMD_PWM_CONTROL_MODE)
    
    #Control Goals
    def setGoalCurrent(self, current):
        value=motor.mAmpsToNumber(current)
        self.writeAddress(ADDR_PRO_GOAL_CURRENT,value,LEN_PRO_GOAL_CURRENT)
    def setGoalPositionAngle(self,positionAngle):
        value=motor.angleToNumber(positionAngle)
        self.writeAddress(ADDR_PRO_GOAL_POSITION,value,LEN_PRO_GOAL_POSITION)
    def setGoalLinearPosition(self,positionInmm):
        self.setGoalPositionAngle(motor.linearToAngle(positionInmm))

    def setGoalPWM(self,pwmPercentage):
        value=motor.dutyCycleToNumber(pwmPercentage)
        self.writeAddress(ADDR_PRO_GOAL_PWM,value,LEN_PRO_GOAL_PWM)

    def setHomeOffset(self,numberPosition):
        self.writeAddress(ADDR_PRO_HOMING_OFFSET,numberPosition,LEN_PRO_HOMING_OFFSET)
    def setHome(self):
        
        pos=self.readAddress(ADDR_PRO_PRESENT_POSITION,LEN_PRO_GOAL_POSITION)
        currentoffset=self.readAddress(ADDR_PRO_HOMING_OFFSET,LEN_PRO_HOMING_OFFSET)
        self.setHomeOffset(currentoffset-pos)
        

    #Read Values
    def getPresentCurrent(self):
        return motor.numberTomAmps(self.readAddress(ADDR_PRO_PRESENT_CURRENT,LEN_PRO_GOAL_CURRENT))
    def getPresentPositionAngle(self):
        number=self.readAddress(ADDR_PRO_PRESENT_POSITION,LEN_PRO_GOAL_POSITION)
        return motor.numberToAngle(number)
    def getPresentPWM(self):
        return motor.numberToDutyCicle(self.readAddress(ADDR_PRO_PRESENT_PWM,LEN_PRO_GOAL_PWM))
    def getPresentLinearposition(self):
        return motor.angleToLinear(self.getPresentPositionAngle())
    def getPresentVelocity(self):
        return   motor.numberToradS(self.readAddress(ADDR_PRO_PRESENT_VELOCITY,LEN_PRO_GOAL_VELOCITY))




            


    def keyBinds(self,e):
     
        if( e.name=='d' ):          
            self.disableTorque()
        elif(e.name==('a')):
            self.enableTorque()
        elif(e.name==('h')):
            self.setHome()
        elif(e.name==('p')):
            print(self.getPresentPositionAngle())
            
           

    def isArmed(self):
        return self.readAddress(ADDR_PRO_TORQUE_ENABLE,LEN_PRO_TORQUE_ENABLE)







