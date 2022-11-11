#COMMANDS
CMD_TORQUE_ENABLE               = 1                 # Value for enabling the torque
CMD_TORQUE_DISABLE              = 0                 # Value for disabling the torque
CMD_EXT_POSITION_CONTROL_MODE   = 4                  #Value for extended position control mode (operating mode)
CMD_CURRENT_CONTROL_MODE = 0
CMD_POSITION_CONTROL_MODE = 3
CMD_VELOCITY_CONTROL_MODE = 1
CMD_CURRENT_BASED_POSITION_CONTROL_MODE = 5
CMD_PWM_CONTROL_MODE = 16




LEN_PRO_TORQUE_ENABLE      = 1              
LEN_OPERATING_MODE=1

LEN_PRO_GOAL_POSITION      = 4
LEN_PRO_GOAL_CURRENT=2
LEN_PRO_GOAL_VELOCITY=4
LEN_PRO_GOAL_PWM=2
LEN_PRO_HOMING_OFFSET=4