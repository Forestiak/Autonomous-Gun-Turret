import jetson.inference
import jetson_utils
import cv2

net = jetson.inference.detectNet("SSD-Mobilenet-v2", threshold = 0.5)

cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

while True:
    success, img = cap.read()
    imgCuda = jetson.utils.cudaFromNumpy(img)
    detections = net.Detect(imgCuda)
    for d in detections:
            # Create bounding box
        x1,y1,x2,y2 = int(d.Left),int(d.Top),int(d.Right),int(d.Bottom)
        className = net.GetClassDesc(d.ClassID)
        cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,255),2)
        cv2.putText(img, className, (x1+5,y1+5),cv2.FONT_HERSHEY_DUPLEX,0.75,(255,0,255),2)
            # Only do distance calculations for people, (ClassID 1 = person)
        if d.ClassID == 1:
            # Center of the bounding box y-coordinate / x-coordinate 
            xAxis,yAxis = int((d.Left+d.Right)/2),int((d.Top+d.Bottom)/2)
            # Distance to the center of the camera screen
            xDist,yDist = int((d.Left+d.Right)/2)-320,int((d.Top+d.Bottom)/2)-240

            cv2.line(img,(xAxis,yAxis),(320,240),(0,255,0),3)
            #print(xDist,yDist)
        pass
    cv2.imshow("Image", img)
    cv2.waitKey(1)
