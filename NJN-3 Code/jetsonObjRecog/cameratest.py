import jetson.inference
import jetson.utils
import cv2



net = jetson.inference.detectNet("SSD-mobilenet-v2", threshold = 0.5)

cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

while True:
    success, img = cap.read()
    imgCuda = jetson.utils.cudaFromNumpy(img)

    detections = net.Detect(imgCuda)
    img = jetson.utils.cudaToNumpy(imgCuda)

    cv2.imshow("Image", img)
    cv2.waitKey(1)