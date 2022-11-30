import SSDMobileNetModule as mnSSDm
import cv2
import motorModule as mM

def main():
    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)
    myModel = mnSSDm.mnSSD("ssd-mobilenet-v2", 0.5)

    while True:
        success, img = cap.read()
        objects = myModel.detect(img, True)[0]
        positionMotor = myModel.detect(img,True)[1]
        cv2.imshow("Image", img)
        cv2.waitKey(1)
        mM.motorModules.move(positionMotor)



if __name__ == "__main__":
    main()
