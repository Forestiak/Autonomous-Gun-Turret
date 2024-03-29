import rospy
import time
import os
from sensor_msgs.msg import LaserScan

once = False
lidar0 = 360*[None]
lidar1 = 360*[None]
values = []
count = 0
total = 0

def callback(msg):

	global total
	global count
	fixes = 0
	global once
	tolerance = 0.2
	group_range = 1
	values = []
	os.system('clear')

	if once == False:
		for i in range(0,180):
			i = 90 + i
			lidar0[i]=round(msg.ranges[i],1)
		once = True

	count = count + 1

	for i in range(0,180):
        	i = 90 + i
		lidar1[i]=round(msg.ranges[i],1)

	for i in range(0,180):
        	i = 90 + i
		if abs(lidar0[i] - lidar1[i]) > tolerance:
            		lidar1[i]=1
        	else:
             		lidar1[i]=0

	for i in range(0,180):
        	i = 90 + i
		if lidar1[i] == 1:
                	kill = 0
			for r in range(-group_range,group_range):
                    		if lidar1[i + r] == 1:
                       			kill = kill + 1
			if kill < group_range:
                    		lidar1[i] = 0
				fixes = fixes + 1
				total = total + 1

	i = 0
	ii = 0
	while i <= 180:
		i = i + 1
		if lidar1[i + 90] == 1:
			ii = i
			while lidar1[i + 90] == 1:
				i = i + 1
				if lidar1[i + 91] == 0:
					kill = 0
					for r in range(2,group_range + 2):
						if lidar1[i + 90 + r] == 1:
							kill = kill + 1
					if kill >= group_range - 1: 
						lidar1[i + 91] = 1
						print 'correction made'
			x = 0
			x = (ii + i - 1) / 2
			values.append(x)
	
	print values
	print 'Amount of noise detected = ',fixes
	print 'Average noise = ',total/count
	return values

rospy.init_node('scan_values')
sub=rospy.Subscriber('/scan',LaserScan,callback)
rospy.spin()
