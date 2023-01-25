import rospy
import time
import os
from sensor_msgs.msg import LaserScan

#Declare global:

once = False
lidar0 = 360*[None]
lidar1 = 360*[None]
values = []
count = 0
total = 0

def callback(msg):

	#Declare local:

	global total
	global count
	fixes = 0
	global once
	tolerance = 0.2
	group_range = 1
	values = []
	os.system('clear')

	#Scan the room for the first time and save it into lidar0

	if once == False:
		for i in range(0,180):
			i = 90 + i
			lidar0[i]=round(msg.ranges[i],1)
			#time.sleep(0.1)
		once = True

	#Scan the room again to compare and save it into lidar1

	count = count + 1

	for i in range(0,180):
        	i = 90 + i
		lidar1[i]=round(msg.ranges[i],1)
        	#time.sleep(0.1)

	#Compare both arrays, if an angle changed more than the tolerance, convert it into a 1, otherwise into 0 (lidar1)

	for i in range(0,180):
        	i = 90 + i
		if abs(lidar0[i] - lidar1[i]) > tolerance:
            		lidar1[i]=1
        	else:
             		lidar1[i]=0
        	#print 'Angle',i,' = ',lidar1[i]

	#Clear noise by converting alone points into 0 (Scales with group_range)

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
           	#print 'Angle',i,' = ',lidar1[i]


	#Locate each separate object and calculate its center

	i = 0
	ii = 0
	while i <= 180: #From 0º to 180º
		i = i + 1
		if lidar1[i + 90] == 1: #If it locates a 1, save its position to later calculate the center of a possible object
			ii = i
			while lidar1[i + 90] == 1: #Continue while there is a 1
				i = i + 1
				if lidar1[i + 91] == 0: #If there is a 0:
					kill = 0
					for r in range(2,group_range + 2): #Decide if it is an error or the end of the object
						if lidar1[i + 90 + r] == 1:
							kill = kill + 1
					if kill >= group_range - 1: #Correct the error if needed
						lidar1[i + 91] = 1
						print 'correction made'
			x = 0
			x = (ii + i - 1) / 2
			values.append(x)
	
  #Print results
  
	print values
	print 'Amount of noise detected = ',fixes
	print 'Average noise = ',total/count
	#time.sleep(1)
	return values

rospy.init_node('scan_values')
sub=rospy.Subscriber('/scan',LaserScan,callback)
rospy.spin()
