from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
import os
import math
from Shrink_and_expand import shrink_procedure

if __name__ == "__main__":
    shrink_procedure()
    # expand_procedure()
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, 0)