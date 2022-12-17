#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
	#Prints length of Lidar
	#print(len(msg.ranges))

	#Prints information at the number of degrees
	print(msg.ranges[180])


rospy.init_node('scan_values')
sub=rospy.Subscriber('/scan',LaserScan,callback)
rospy.spin()
