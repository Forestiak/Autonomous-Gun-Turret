import jetson.inference
import jetson.utils
import cv2
import motorModule
xDegree = 0
yDegree = 0
class mnSSD():
    def __init__(self,path,threshold):
        self.path = path
        self.threshold = threshold
        self.net = jetson.inference.detectNet(self.path, self.threshold)

    def detect(self, img, display = False):
        imgCuda = jetson.utils.cudaFromNumpy(img)
        detections = self.net.Detect(imgCuda, overlay = "OVERLAY_NONE")
        objects = []
        for d in detections:
            className = self.net.GetClassDesc(d.ClassID)
            objects.append([className, d])
            if display:
                x1,y1,x2,y2 = int(d.Left),int(d.Top),int(d.Right),int(d.Bottom)
                cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,255),2)
                cv2.putText(img, className, (x1+5,y1+15),cv2.FONT_HERSHEY_DUPLEX,0.75,(255,0,255),2)
                if d.ClassID == 1:
                    global xDegree
                    global yDegree
                    # Center of the bounding box y-coordinate / x-coordinate 
                    xAxis,yAxis = int((d.Left+d.Right)/2),int((d.Top+d.Bottom)/2)
                    # Distance to the center of the camera screen
                    xDegree,yDegree = int(((d.Left+d.Right)/2)-320),int(((d.Top+d.Bottom)/2)-240)
                    cv2.line(img,(xAxis,yAxis),(320,240),(0,255,0),3)
                else: 
                    xDegree = 0
                    yDegree = 0
        return objects, xDegree, yDegree
