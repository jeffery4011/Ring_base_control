from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
import os
import math
#import keyboard

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

from dynamixel_sdk import * # Uses Dynamixel SDK library
#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V

Max_Ring_length = 10000    #14000 # unit 0.1mm
Min_Ring_length = 5800
radius = 98
round_value = 4096


# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
    ADDR_PRO_PRESENT_LOAD       = 126
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
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
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
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

def motor_center(x,mx,error):
    if (x>mx//2+error):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, +5)
        #print('Left')
        return
    if (x<mx//2-error):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, -5)
        #print('Left')
        return
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, 0)

camera = PiCamera()
camera.framerate = 32
rawCapture = PiRGBArray(camera)
name = "Object!"
maxObjects = 1
minObjectArea = 50
blurSize = 9                        # Blur Kernel Size
morphOpSize = 5                     # Closing and Opening Kernel Size

#lower= np.array([160,100,120],np.uint8) # Array (H,S,V) for the lower threshold bound of the HSV image
#upper= np.array([180,120,255],np.uint8) # Array (H,S,V) for the upper threshold bound of the HSV image
lower= np.array([0,125,125],np.uint8) # Array (H,S,V) for the lower threshold bound of the HSV image
upper= np.array([100,255,255],np.uint8) # Array (H,S,V) for the upper threshold bound of the HSV image

error = np.array([13,100,100],np.uint8) # Array of error widths to create the upper and lower threshold bounds above.

def drawCOM(frame, x, y, name):
    cv2.circle(frame,(x,y),5,(0,255,0))
    cv2.putText(frame,name,(x-30,y-25),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,0),2)

def findObjects(binaryMatrix):
    #Finds the location of the desired object in the image.
    output = []
    contours, hierarchy = cv2.findContours(binaryMatrix, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Contours the image to find blobs of the same color   
    
    cont = sorted(contours, key = cv2.contourArea, reverse = True)[:maxObjects]                   # Sorts the blobs by size (Largest to smallest) 

    # Find the center of mass of the blob if there are any
    if hierarchy  is not None:
        for i in range (0,len(cont)):
            M = cv2.moments(cont[i])
            if M['m00'] > minObjectArea:                                   # Check if the total area of the contour is large enough to care about!
                rect = cv2.minAreaRect(cont[0])
                w = int(rect[1][0])
                x = int(M['m10']/M['m00'])
                y = int(M['m01']/M['m00'])
		#if(debug):                
                cv2.drawContours(imgTrack, cont[i], -1, (255,255,255), 3) # Draws the contour.
                drawCOM(imgTrack,x,y,name)
                cv2.imshow("Frame",imgTrack)
                
                if output == []:
                    output = [[x,w]]
                else:
                    output.append[[x,w]]
    return output

def morphOps(binaryMatrix, kernelSize):
    # Morphological operations (open and close) used to reduce noise in the acquired image.
    kernel = np.ones((kernelSize,kernelSize), np.uint8)
    tempFix = cv2.morphologyEx(binaryMatrix,cv2.MORPH_CLOSE, kernel)   # Fill in holes
    fix = cv2.morphologyEx(tempFix,cv2.MORPH_OPEN, kernel)             # Get rid of noise
    return fix

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture,format = "bgr",use_video_port = True):
        image = frame.array
        image = cv2.rotate(image,cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow("Frame",image)
        imgBGR = image

    # height and width of the image to pass along to the PID controller as the reference point.
        height, width = imgBGR.shape[:2]

    # Image used to draw things on!
        imgTrack = imgBGR.copy()
    
    # # Blur the image to reduce edges caused by noise or that are useless to us.
    #     imgBlur = cv2.GaussianBlur(imgBGR,(blurSize,blurSize),0)
    #     #cv2.imshow("gassianed",imgBlur)
    # # Transform BGR to HSV to avoid lighting issues.
    #     imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)	
    
    # # Threshold the image using the selected lower and upper bounds of the color of the object.
    #     mask = cv2.inRange(imgHSV, lower, upper)
    #     cv2.imshow("masked",mask)
    # # To get rid of noise and fill in gaps in our object use open and close.
    #     imgMorphOps = morphOps(mask, morphOpSize)


    #     centers = findObjects(imgMorphOps)
        #print(centers)
        #print(width)
        # if len(centers)==0:
        #         centers.append([width//2,width//2])
        #motor_center(centers[0][0],width,width/20)
        key = cv2.waitKey(1)
        rawCapture.truncate(0)
        if key == ord("a"):
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, +5)
        if key == ord("d"):
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, -5)
        if key == ord("q"):
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, 0)
        if key == ord("s"):
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, -265)
                print("You pressed s ")
        if key == ord("e"):
                print("You pressed e ")
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, 265)
        if key == ord("i"):
                print("You pressed i ")
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, 0)
