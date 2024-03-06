from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
import os
import math

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * 
MY_DXL = 'X_SERIES'
Max_Ring_length = 10000    #14000 # unit 0.1mm
Min_Ring_length = 5800
radius = 98
round_value = 4096

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    global ADDR_TORQUE_ENABLE          
    ADDR_TORQUE_ENABLE          = 64
    global ADDR_GOAL_POSITION          
    ADDR_GOAL_POSITION          = 116
    global ADDR_PRESENT_POSITION      
    ADDR_PRESENT_POSITION       = 132
    global DXL_MINIMUM_POSITION_VALUE          # Refer to the Minimum Position Limit of product eManual
    DXL_MINIMUM_POSITION_VALUE  = 0
    global DXL_MAXIMUM_POSITION_VALUE        # Refer to the Maximum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095
    global BAUDRATE                    
    BAUDRATE = 57600
    global ADDR_PRO_PRESENT_LOAD
    ADDR_PRO_PRESENT_LOAD       = 126
    global ADDR_GOAL_VELOCITY
    ADDR_GOAL_VELOCITY = 104

PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 2

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = "/dev/ttyUSB0"

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 30    # Dynamixel moving status threshold

index = 0

global portHandler 
portHandler = PortHandler(DEVICENAME)
global packetHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

global shrink_limit 
shrink_limit = 0
global expand_limit 
expand_limit = 0
global shrink_jamming_limit
shrink_jamming_limit = -400
global expand_jamming_limit 
expand_jamming_limit = 400
global time_limit 
time_limit = 2e4
global jamming_stop_limit_time 
jamming_stop_limit_time = 20

def shrink_procedure():
    print('Shrink')
    global time_limit
    global portHandler 
    global packetHandler
    global ADDR_TORQUE_ENABLE          
    global ADDR_GOAL_POSITION          
    global ADDR_PRESENT_POSITION       
    global DXL_MINIMUM_POSITION_VALUE 
    global DXL_MAXIMUM_POSITION_VALUE  
    global BAUDRATE                    
    global ADDR_PRO_PRESENT_LOAD       
    global ADDR_GOAL_VELOCITY 
    global shrink_jamming_limit
    global time_limit
    current_load ,dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, 1, ADDR_PRO_PRESENT_LOAD)
    limit_reach = 0
    time = 0
    while limit_reach<jamming_stop_limit_time and time < time_limit :
        time += 1
        if current_load>32768:
            current_load = current_load-65536
        print('Current_load')
        print(current_load)
        print('Time:')
        print(time)
        if current_load <= shrink_jamming_limit:
            limit_reach +=1
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, -265)
        current_load ,dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, 1, ADDR_PRO_PRESENT_LOAD)
        if current_load>32768:
            current_load = current_load-65536
    return True

def expand_procedure():
    print('Expand')
    global portHandler 
    global packetHandler
    global ADDR_TORQUE_ENABLE          
    global ADDR_GOAL_POSITION          
    global ADDR_PRESENT_POSITION       
    global DXL_MINIMUM_POSITION_VALUE 
    global DXL_MAXIMUM_POSITION_VALUE  
    global BAUDRATE                    
    global ADDR_PRO_PRESENT_LOAD       
    global ADDR_GOAL_VELOCITY 
    global expand_jamming_limit
    global time_limit
    current_load ,dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, 1, ADDR_PRO_PRESENT_LOAD)
    if current_load>32768:
        current_load = current_load-65536
    limit_reach = 0
    time = 0
    while limit_reach<jamming_stop_limit_time and time < time_limit :
        print('Current_load')
        print(current_load)
        time +=1
        print('Time:')
        print(time)
        if current_load >= expand_jamming_limit:
            limit_reach +=1
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, 265)
        current_load ,dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, 1, ADDR_PRO_PRESENT_LOAD)
        if current_load>32768:
            current_load = current_load-65536
    return True

def One_cycle_Shrink_Expand():
    expand_procedure()
    shrink_procedure()
    
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, 0)

def All_stop():
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, 0)

if __name__ == "__main__":
    shrink_procedure()
    expand_procedure()
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, 0)
    

