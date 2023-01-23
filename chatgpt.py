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

case = 0 #fsm
laserpin = None #laser pin
anglex = 90 # Angle which the turret is facing
angley = 90
#1 step = 0.9 degrees


#motors

directionX = 0
directionY = 0
GPIO.cleanup()
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
    #Move the motor during a number of steps
    def move_MotorX(stepsX,speed):
        # Set the first direction you want it to spin
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
        for x in range(stepsX // 20):
                GPIO.output(STEPH, GPIO.HIGH)
                sleep(speed)
                GPIO.output(STEPH, GPIO.LOW)
                sleep(speed)



    def move_MotorY(stepsY,speed):
        # Set the first direction you want it to spin
        global directionY
        if stepsY > 0:
            directionY = CW
        elif stepsY < 0:
            directionY = CCW
            stepsY = abs(stepsY)
        else:
            stepsY = 0
        print(stepsY)
        GPIO.output(DIRV, directionY)
        for x in range(stepsY // 20):
                GPIO.output(STEPV, GPIO.HIGH)
                sleep(speed)
                GPIO.output(STEPV, GPIO.LOW)
                                sleep(speed)

# parse the command line
parser = argparse.ArgumentParser(description="Run pose estimation DNN on a video/image stream.", 
                                 formatter_class=argparse.RawTextHelpFormatter, 
                                 epilog=poseNet.Usage() + videoSource.Usage() + videoOutput.Usage() + logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="resnet18-body", help="pre-trained model to use")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)")
parser.add_argument("--alpha", type=float, default=0.5, help="blending alpha (transparency)")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use")
parser.add_argument("--camera", type=str, default="0", help="index of the MIPI CSI camera to use (e.g. CSI camera 0)\nor for VL42 cameras, the /dev/video device to use.\nby default, MIPI CSI camera 0 will be used.")
parser.add_argument("--width", type=int, default=1280, help="desired width of camera stream (default is 1280 pixels)")
parser.add_argument("--height", type=int, default=720, help="desired height of camera stream (default is 720 pixels)")

# parse the command line
opt = parser.parse_args()

# load the recognition model
net = poseNet.create(opt.network, opt.overlay, opt.threshold)

# open the video source
source = videoSource.create(opt.input_URI, opt.camera, opt.width, opt.height)

# open the output stream
out = videoOutput.create(opt.output_URI, source.get(cv2.CAP_PROP_FPS))

# create a window to display the camera feed
cv2.namedWindow("Stream", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Stream", opt.width, opt.height)

# Allocate a CUDA context for the DNN
cuda_context = cudaAllocMapped(size=1e9)

# Process frames until the video ends, or process is exited
while True:
    # Capture the next frame
    ret, img = source.read()
    if not ret:
        break

    # Convert the color format of the frame
    img = cudaConvertColor(img, cv2.COLOR_BGR2RGBA)

    # Allocate CUDA memory for the frame
    img_cuda = cudaFromNumpy(img)

       # Execute the DNN
    net.imageSize(img.shape[1], img.shape[0])
    net.overlayThreshold(opt.threshold)
    net.enableProfiler()
    detections = net.detect(img_cuda)
    net.disableProfiler()

    # Get the recognized faces
    face_locations = face_recognition.face_locations(img)

    # Draw boxes around the recognized faces
    for (top, right, bottom, left) in face_locations:
        cv2.rectangle(img, (left, top), (right, bottom), (0, 0, 255), 2)

    # Convert the frame back to color format
    img = cudaToNumpy(img_cuda, img.shape[0], img.shape[1], img.shape[2])
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

    # Write the output frame
    out.write(img)

    # Display the output
    cv2.imshow("Stream", img)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# Cleanup
source.release()
out.release()
cv2.destroyAllWindows()


