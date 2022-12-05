#!/usr/bin/env python3
#
# Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

import sys
import argparse
import cv2
import numpy
from jetson_inference import poseNet
from jetson_utils import (cudaFromNumpy, cudaAllocMapped, cudaConvertColor, 
                          cudaDeviceSynchronize, saveImage, cudaToNumpy)
from jetson_utils import videoSource, videoOutput, logUsage
import os
import pickle
import face_recognition
from PIL import Image 


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
                if True in matches:
                    first_match_index=matches.index(True)
                    name=Names[first_match_index]
                    
                    #cv2.rectangle(testImage,(left,top),(right,bottom),(0,0,255),2)
                #cv2.putText(testImage,name,(left,top-6),font,.75,(0,255,255),2) 
                
                print(name , " is at x:",centerx , ", y:" , centery)
        #label = face_recognizer.predict(face)
        #pose.ID = label[0]
        #print(pose.ID)
        # print(pose.Keypoints)
        #print('Links', pose.Links)

    if len(poses)>0:
        cv2.imshow("Image", rec)
    else:
        cv2.imshow("Image", img)
    cv2.waitKey(1)
