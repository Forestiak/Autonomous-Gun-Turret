import rospy
import time
import os
from sensor_msgs.msg import LaserScan
import sys
import argparse
import cv2
import numpy
from jetson_inference import poseNet
from jetson_utils import (cudaFromNumpy, cudaAllocMapped, cudaConvertColor, 
                          cudaDeviceSynchronize, saveImage, cudaToNumpy)
from jetson_utils import videoSource, videoOutput, logUsage
import pickle
import face_recognition
from PIL import Image
import RPi.GPIO as GPIO
from time import sleep
import SSDMobileNetModule as sm
import math
import queue


laserpin = 3 #laser pin
anglex = 90 # Angle which the turret is facing
angley = 90
#1 step = 0.9 degrees

#lidar variables:

once = False
lidar0 = 360*[None]
lidar1 = 360*[None]

coincidence = 25

#motors

directionX = 0
directionY = 0
# Direction pin from controller
DIRH = 7   
DIRV = 11
# Step pin from controller
STEPH = 33
STEPV = 32
# 0/1 used to signify clockwise or counterclockwise.
CW = 1
CCW = 0
GPIO.setwarnings(False)
# Setup pin layout on PI
GPIO.setmode(GPIO.BOARD)
# Establish Pins in software
GPIO.setup(laserpin, GPIO.OUT)
GPIO.setup(DIRH, GPIO.OUT)
GPIO.setup(STEPH, GPIO.OUT)
GPIO.setup(DIRV, GPIO.OUT)
GPIO.setup(STEPV, GPIO.OUT)
# Set the first direction you want it to spin
GPIO.output(DIRH, CW)
GPIO.output(DIRV, CW)
class motorModules():
    def __init__(self):
        self.pwmH = GPIO.PWM(STEPH, 50)
        self.pwmV = GPIO.PWM(STEPV, 50)
    
    def move_MotorX(self,stepsX,speed):
        global directionX
        # This is dependant on which way the A and B Ports on the motor driver are connected
        if stepsX < 0:
            directionX = CW
            stepsX = abs(stepsX)
        elif stepsX > 0:
            directionX = CCW
        else:
            stepsX = 0
        GPIO.output(DIRH, directionX)
        self.pwmH.start(speed)
        self.pwmH.ChangeDutyCycle(speed)
        sleep(abs(stepsX / 20))
        self.pwmH.stop()

    def move_MotorY(self,stepsY,speed):
        global directionY
        if stepsY > 0:
            directionY = CW
        elif stepsY < 0:
            directionY = CCW
            stepsY = abs(stepsY)
        else:
            stepsY = 0
        GPIO.output(DIRV, directionY)
        self.pwmV.start(speed)
        self.pwmV.ChangeDutyCycle(speed)
        sleep(abs(stepsY / 20))
        self.pwmV.stop()

# your code
GPIO.cleanup()

# parse the command line
parser = argparse.ArgumentParser(description="Run pose estimation DNN on a video/image stream.", 
                                 formatter_class=argparse.RawTextHelpFormatter, 
                                 epilog=poseNet.Usage() + videoSource.Usage() + videoOutput.Usage() + logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="resnet18-body", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="links,keypoints", help="pose overlay flags (e.g. --overlay=links,keypoints)\nvalid combinations are:  'links', 'keypoints', 'boxes', 'none'")
parser.add_argument("--threshold", type=float, default=0.15, help="minimum detection threshold to use") 

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# load the pose estimation model
net = poseNet(opt.network, sys.argv, opt.threshold)

# create video sources & outputs
cap = cv2.VideoCapture(0)
#output = videoOutput(opt.output_URI, argv=sys.argv)

# process frames until the user exits
right1 = 0
left1 = 0
top = 0
bottom = 0
j=0
Encodings=[]
Names=[]
    
scale=.7

pi=False
pf=False

def distance(x,y,):
    rangex=(640-x)^2
    rangey=(360-y)^2
    
    range= math.sqrt(rangex+rangey)
    
    targrange=math.sqrt(((640-xtarget)^2)+((360-ytarget)^2))

    if range> targrange:
        return False
    else:
        return True

def motorlimitx (x):
    if x + anglex > 180:
        x = 180 - anglex
    if x + anglex < 0:
        x = anglex - 180

    return x

def motorlimity (y):
    if y + angley > 45:
        y = 45 - angley
    if y + angley < 0:
        y = angley - 45

    return y

with open('/home/aau/PyPro/faceRecognizer/train.pkl','rb') as f:
	Names=pickle.load(f)
	Encodings=pickle.load(f)

while True:
    # capture the next image
    success, img = cap.read()
    #img1=img
    bgr_img = cudaFromNumpy(img, isBGR=True)
    rgb_img = cudaAllocMapped(width=bgr_img.width,
                          height=bgr_img.height,
						  format='rgb8')
    cudaConvertColor(bgr_img, rgb_img)

    # perform pose estimation (with overlay)
    poses = net.Process(rgb_img, overlay=opt.overlay)

    # print the pose results
    #print("detected {:d} objects in image".format(len(poses)))

    img = cudaToNumpy(rgb_img)
   # frameSmall=cv2.resize(img,(0,0),fx=scale,fy=scale)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  
    xtarget = 2000
    ytarget = 2000


    for pose in poses:
        if pose.FindKeypoint('left_ear') < 0:
            left1 = int(pose.FindKeypoint('left_eye'))
        else:
            left1 = int(pose.FindKeypoint('left_ear'))
        if pose.FindKeypoint('right_ear') < 0:
            right1 = int(pose.FindKeypoint('right_eye'))
        else:
            right1 = (pose.FindKeypoint('right_ear'))

        left1 = pose.Keypoints[left1]
        right1 = pose.Keypoints[right1]

        inc = (1.618*abs(right1.x - left1.x))/2
        left1.y = left1.y + inc
        right1.y = right1.y - inc
        #print(left.x, left.y, right.x, right.y)
        rec = cv2.rectangle(img, (int(left1.x), int(left1.y)), (int(right1.x), int(right1.y)), color = (255, 0, 0), thickness=2)
        
       
        
        _,frame=cap.read()
        

        frameRGB=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)


        #Rectangle marker
        rect_img = frame[(int(right1.y)+10):(int(left1.y)-10), (int(right1.x)+10):(int(left1.x)-10)]
        
        #sketcher_rect = rect_img
        #sketcher_rect = sketch_transform(sketcher_rect)
        
        #Conversion for 3 channels to put back on original image (streaming)
        #sketcher_rect_rgb = cv2.cvtColor(sketcher_rect, cv2.COLOR_BGR2RGB)
        
        #Replacing the sketched image on Region of Interest
        #frame[bottom_right[1]: upper_left[1]  , bottom_right[0] : upper_left[0]] = sketcher_rect_rgb
        if rect_img.any() != False:
            
            frameSmall=cv2.resize(rect_img,(0,0),fx=scale,fy=scale)
            
            facePositions=face_recognition.face_locations(frameSmall, model='cnn')
            allEncodings=face_recognition.face_encodings(frameSmall, facePositions)
           
            centery=(int(right1.y)+int(left1.y))/2
            centerx=(int(right1.x)+int(left1.x))/2
            
            for (top,right,bottom,left),face_encoding in zip(facePositions, allEncodings):
                
                matches=face_recognition.compare_faces(Encodings,face_encoding)
                name='Unknown person'
                pi=False
                if True in matches:
                    first_match_index=matches.index(True)
                    name=Names[first_match_index]
                    pi=True
                    #cv2.rectangle(testImage,(left,top),(right,bottom),(0,0,255),2)
                #cv2.putText(testImage,name,(left,top-6),font,.75,(0,255,255),2) 
                
                if pi==True | pi==pf: #if the person is priority or the objective isnt
                    if distance(centerx,centery)==True: #if the person is closer
                        xtarget=centerx
                        ytarget=centery 

                print(name , " is at x:",centerx , ", y:" , centery)
        GPIO.output(laserpin, GPIO.HIGH)
        if pf == False:
            GPIO.output(laserpin, GPIO.LOW)
        #pixels --> steps
        xtarget = max(1,round((xtarget*0.9)-5))
        ytarget = max(1,round((ytarget*0.9)-5))
        #motor limit
        xtarget = motorlimitx (xtarget)
        ytarget = motorlimity (ytarget)
        #update angle(s)
        angley = angley + ytarget
        anglex = anglex + xtarget
        #move motors based on camera input
        motorModules.move_MotorX(xtarget,0.0005)
        motorModules.move_MotorY(ytarget,0.0005)
        GPIO.output(laserpin, GPIO.LOW)
      
    if len(poses)>0:
        cv2.imshow("Image", rec)
    else:
        cv2.imshow("Image", img)
    cv2.waitKey(1)

   
