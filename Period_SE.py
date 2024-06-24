from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
import os
import math
# from Shrink_and_expand import shrink_procedure
# from Shrink_and_expand import All_stop
from Shrink_and_expand import expand_procedure_determined_time
from Shrink_and_expand import shrink_procedure_determined_time

if __name__ == "__main__":
    # shrink_procedure()
    # # expand_procedure()
    # # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, 0)
    # All_stop()
    time= 100
    while True:
        shrink_procedure_determined_time(time)
        expand_procedure_determined_time(time)