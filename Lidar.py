#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

#This code requires the laser node to be initializated which can be achieved by the command
#roslaunch hls_lfcd_lds_driver hlds_laser.launch


def callback(msg):
	#Prints length of Lidar
	#print(len(msg.ranges))

	#Prints information at the number of degrees specified
	print(msg.ranges[180])


rospy.init_node('scan_values')
sub=rospy.Subscriber('/scan',LaserScan,callback)
rospy.spin()
