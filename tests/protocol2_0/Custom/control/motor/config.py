# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
CURRENT_LIMIT_MIN=0
CURRENT_LIMIT_MAX=648
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4095            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
# Default setting
               # Dynamixel ID : 1
            # Dynamixel default baudrate : 57600
   # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
