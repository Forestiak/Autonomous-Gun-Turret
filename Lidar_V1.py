import rospy
import time
import os
from sensor_msgs.msg import LaserScan

#Declare global:

once = False
lidar0 = 360*[None]
lidar1 = 360*[None]
values = []

def callback(msg):

	#Declare local:

	global once
	tolerance = 0.2
	group_range = 7
	values = []
	os.system('clear')

	#Scan the room for the first time and save it into lidar0

	if once == False: 
		for i in range(0,180):
			i = 90 + i
			lidar0[i]=round(msg.ranges[i],1) #scaning the room and saving it
		once = True #make sure this code only runs once by using a boolean variable
	

	#Scan the room again to compare and save it into lidar1

	for i in range(0,180):
        	i = 90 + i
		lidar1[i]=round(msg.ranges[i],1) #scaning the room and saving it

	#Compare both arrays, if an angle changed more than the tolerance, convert it into a 1, otherwise into 0 (lidar1)

	for i in range(0,180):
        	i = 90 + i
		if abs(lidar0[i] - lidar1[i]) > tolerance: #Run the following code if the difference beween both arrays is bigger than the tolerance (20cm)
			#convert lidar1 into a true/false array:
            		lidar1[i]=1 
        	else:
             		lidar1[i]=0

	#Clear noise by converting alone points into 0 (Scales with group_range)

	for i in range(0,180):
        	i = 90 + i
		if lidar1[i] == 1: #if a true is detected
                	kill = 0
			for r in range(-group_range,group_range): #scan in its vicinity
                    		if lidar1[i + r] == 1:
                       			kill = kill + 1 #count the trues around it
			if kill < group_range: #if there are too many falses around -> its noise -> convert it to false
                    		lidar1[i] = 0


	#Locate each separate object and calculate its center

	i = 0
	ii = 0
	while i <= 180: #Scan 180º
		i = i + 1
		if lidar1[i + 90] == 1: #if it encounters something
			ii = i #save first coordinate
			while lidar1[i + 90] == 1: #while the object continues
				i = i + 1
				if lidar1[i + 91] == 0: #if a limit is reached
					kill = 0
					for r in range(2,group_range): #make sure it isnt noise by looking ahead
						if lidar1[i + 90 + r] == 1:
							kill = kill + 1
					if kill >= group_range - 1: #if its noise -> correct it
						lidar1[i + 91] = 1
			x = 0
			x = (ii + i - 1) / 2 #calculate center of the object
			values.append(x) #add the object to a queue
	
	#print values
	#time.sleep(1)
	return values

rospy.init_node('scan_values') #make new ROS node called scan_values responsible for subscribing to the /scan topic
sub=rospy.Subscriber('/scan',LaserScan,callback) #create a new subscriber for the /scan topic. This subscriber will receive messages on the /scan topic and pass them to the callback function for further processing.
rospy.spin() #tell the node to enter a loop where it will process any incoming messages on the /scan topic, passing them to the callback function as they are received. This loop will run until the node is shut down or the program is terminated.



