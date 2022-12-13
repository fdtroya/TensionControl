
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
    bulkConversion=np.array([0.01356,2.69,0.023981,0.087912087912])

    def toSigned32(n):
        n = n & 0xffffffff
        return (n ^ 0x80000000) - 0x80000000

    def mAmpsToNumber(mAmps): #0-1743.12
        return round(mAmps/motor.bulkConversion[1])
    def numberTomAmps(number):
        return number*motor.bulkConversion[1]
    def angleToLinear(angle):
        return angle*(motor.displacementPerRotation/360)
    def linearToAngle(linear):
        return 360*(linear/motor.displacementPerRotation)
    def numberToAngle(number):
        return number*motor.bulkConversion[3]
    def angleToNumber(angle):
        return round(angle/motor.bulkConversion[3])
    
    def numberToDutyCicle(number):
        return number*motor.bulkConversion[0]
    def dutyCycleToNumber(dutyCycle):
        return round(dutyCycle/motor.bulkConversion[0])
    def numberToradS(number):
        return number*motor.bulkConversion[2]
    def radSToNumber(radS):
        return round(radS/motor.bulkConversion[2])
    
    def twos_comp(val, bytes):
        bits=8*bytes
        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val 


    def __init__(self,port,BAUDRATE) -> None:
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.BAUDRATE= BAUDRATE
        self.comLock=Lock()
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)
        self.connected=False
        
      
        
    def connect(self,IDS):
        if(not self.connected):

            # Open port
            if self.portHandler.openPort():
                print("Succeeded to open the port")
            else:
                raise Exception("Failed to open the port")

            # Set port baudrate
            if self.portHandler.setBaudRate(self.BAUDRATE):
                print("Succeeded to change the baudrate")
                for id in IDS:
                    self.disableTorque(id)
                self.connected=True
            else:
                raise Exception("Failed to change the baudrate")
        
        

    def disConnect(self):
         # Disable Dynamixel Torque
        self.disableTorque()
        # Close port
        self.portHandler.closePort()
        
    def homming(self,DXL_ID,direction=-1):
        threshold=85
        self.disableTorque(DXL_ID)
        self.setVelocityControlMode(DXL_ID)
        self.enableTorque(DXL_ID)
        self.setGoalVelocity(0.5*direction,DXL_ID)
        time.sleep(0.25)
        current=0
        while(abs(current)<=threshold):
            current=self.getPresentCurrent(DXL_ID)
        print("###########HOME SET###########")
        self.disableTorque(DXL_ID)
        self.setGoalVelocity(0,DXL_ID)
        time.sleep(1)
        self.setHome(DXL_ID)




    def writeAddressInner(self,address,value,byteSize,DXL_ID):
        self.comLock.acquire()
        if(byteSize==1):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, address, value)
        elif(byteSize==2):
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, address, value)
        elif(byteSize==4):
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, address, value)
        else:
            raise Exception("Invalid byteSize")

        if dxl_comm_result != COMM_SUCCESS:
            raise Exception("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            raise Exception("%s" % self.packetHandler.getRxPacketError(dxl_error))
        
        self.comLock.release()
        return True
    def readAddressInner(self,address,byteSize,DXL_ID):
        self.comLock.acquire()
        if(byteSize==1):
            dxl_value, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, DXL_ID, address)
        elif(byteSize==2):
            dxl_value, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_ID, address)
        elif(byteSize==4):
            dxl_value, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, address)
        else:
            raise Exception("Invalid byteSize")
        
        
     
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            raise Exception("%s" % self.packetHandler.getRxPacketError(dxl_error))
        self.comLock.release()
        final=motor.twos_comp(dxl_value,byteSize)
        return final

   

    def readBulkSensors(self,DXL_ID):
        start=124
        end=132
        l=4
        size=end-start+l
        data,result,error=self.packetHandler.readTxRx(self.portHandler,DXL_ID,start,size)
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
        
    def readAddress(self,address,byteSize,DXL_ID):
        count=0
        tries=0
        while True:
            try:    
                return self.readAddressInner(address,byteSize,DXL_ID)
                
            except Exception as e:
                print(str(e))
                if count>=tries:
                    raise(e)
                time.sleep(0.05)
                count+=1

    def writeAddress(self,address,value,byteSize,DXL_ID):
        count=0
        tries=0
        while True:
            try:    
                self.writeAddressInner(address,value,byteSize,DXL_ID)
                return
            except Exception as e:
                print(str(e))  
                if count>=tries:
                    raise(e)
                time.sleep(0.05)
                count+=1
    
            
    def enableTorque(self,DXL_ID):
        if(not self.isArmed(DXL_ID)):
            self.writeAddress(ADDR_PRO_TORQUE_ENABLE,CMD_TORQUE_ENABLE,LEN_PRO_TORQUE_ENABLE,DXL_ID)
         

    def disableTorque(self,DXL_ID):
        if(self.isArmed(DXL_ID)):
            self.writeAddress(ADDR_PRO_TORQUE_ENABLE,CMD_TORQUE_DISABLE,LEN_PRO_TORQUE_ENABLE,DXL_ID) 
            
    #Control Modes
    def setControlMode(self,CMD_MODE,DXL_ID):
        
        self.writeAddress(ADDR_OPERATING_MODE,CMD_MODE,LEN_OPERATING_MODE,DXL_ID)
        

    def setCurrentControlMode(self,DXL_ID):
        self.setControlMode(CMD_CURRENT_CONTROL_MODE,DXL_ID)
    def setPositionControlMode(self,DXL_ID):
        self.setControlMode(CMD_POSITION_CONTROL_MODE,DXL_ID)
    def setExtendedPositionControlMode(self,DXL_ID):
        self.setControlMode(CMD_EXT_POSITION_CONTROL_MODE,DXL_ID)
    def setVelocityControlMode(self,DXL_ID):
        self.setControlMode(CMD_VELOCITY_CONTROL_MODE,DXL_ID)
    def setPWMControlMode(self,DXL_ID):
        self.setControlMode(CMD_PWM_CONTROL_MODE,DXL_ID)
    
    #Control Goals
    def setGoalCurrent(self, current,DXL_ID):
        value=motor.mAmpsToNumber(current)
        self.writeAddress(ADDR_PRO_GOAL_CURRENT,value,LEN_PRO_GOAL_CURRENT,DXL_ID)
    def setGoalPositionAngle(self,positionAngle,DXL_ID):
        value=motor.angleToNumber(positionAngle)
        self.writeAddress(ADDR_PRO_GOAL_POSITION,value,LEN_PRO_GOAL_POSITION,DXL_ID)
    def setGoalVelocity(self,omega,DXL_ID):
        value=motor.radSToNumber(omega)
        self.writeAddress(ADDR_PRO_GOAL_VELOCITY,value,LEN_PRO_GOAL_VELOCITY,DXL_ID)
    def setGoalLinearPosition(self,positionInmm,DXL_ID):
        self.setGoalPositionAngle(motor.linearToAngle(positionInmm),DXL_ID)

    def setGoalPWM(self,voltage,DXL_ID):
        value=motor.dutyCycleToNumber(voltage)
        self.writeAddress(ADDR_PRO_GOAL_PWM,value,LEN_PRO_GOAL_PWM,DXL_ID)

    def setHomeOffset(self,numberPosition,DXL_ID):
        self.writeAddress(ADDR_PRO_HOMING_OFFSET,numberPosition,LEN_PRO_HOMING_OFFSET,DXL_ID)
    def setHome(self,DXL_ID):
        
        pos=self.readAddress(ADDR_PRO_PRESENT_POSITION,LEN_PRO_GOAL_POSITION,DXL_ID)
        currentoffset=self.readAddress(ADDR_PRO_HOMING_OFFSET,LEN_PRO_HOMING_OFFSET,DXL_ID)
        self.setHomeOffset(currentoffset-pos,DXL_ID)
        

    #Read Values
    def getPresentCurrent(self,DXL_ID):
        return motor.numberTomAmps(self.readAddress(ADDR_PRO_PRESENT_CURRENT,LEN_PRO_GOAL_CURRENT,DXL_ID))
    def getPresentPositionAngle(self,DXL_ID):
        number=self.readAddress(ADDR_PRO_PRESENT_POSITION,LEN_PRO_GOAL_POSITION,DXL_ID)
        return motor.numberToAngle(number)
    def getPresentPWM(self,DXL_ID):
        return motor.numberToDutyCicle(self.readAddress(ADDR_PRO_PRESENT_PWM,LEN_PRO_GOAL_PWM,DXL_ID))
    def getPresentLinearposition(self,DXL_ID):
        return motor.angleToLinear(self.getPresentPositionAngle(DXL_ID))
    def getPresentVelocity(self,DXL_ID):
        return   motor.numberToradS(self.readAddress(ADDR_PRO_PRESENT_VELOCITY,LEN_PRO_GOAL_VELOCITY,DXL_ID))




    def isArmed(self,DXL_ID):
        return self.readAddress(ADDR_PRO_TORQUE_ENABLE,LEN_PRO_TORQUE_ENABLE,DXL_ID)            

"""
    def keyBinds(self,e):
     
        if( e.name=='d' ):          
            self.disableTorque()
        elif(e.name==('a')):
            self.enableTorque()
        elif(e.name==('h')):
            self.setHome()
        elif(e.name==('p')):
            print(self.getPresentPositionAngle())
            
   """        









