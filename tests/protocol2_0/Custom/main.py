from motor import *



#position control


dyn1=motor(1,'COM3',57600)
dyn1.connect()
dyn1.setCurrentControlMode()
# Enable Dynamixel Torque 


while True:
    print(dyn1.getPresentLinearposition())
    pos=input("Write  current: ")
    dyn1.setGoalCurrent(int(pos))
        
    
