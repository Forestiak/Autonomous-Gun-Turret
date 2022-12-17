#!/usr/bin/env python

import rospy
import time
import os
from sensor_msgs.msg import LaserScan

def callback(msg):
	#Prints length of Lidar
	#print(len(msg.ranges))

	#Prints information at the number of degrees
        lidar0 = 360*[None]
        os.system('clear')
        for i in range(0,18):
            i = 90 + i*10
            lidar0[i]=round(msg.ranges[i],1)
            print 'Angle',i,' = ',lidar0[i]
        time.sleep(0.1)
        

rospy.init_node('scan_values')
sub=rospy.Subscriber('/scan',LaserScan,callback)
rospy.spin()
