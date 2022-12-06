import RPi.GPIO as GPIO
from time import sleep
import SSDMobileNetModule as sm
directionX = 0
directionY = 0
GPIO.cleanup()
# Direction pin from controller
DIRH = 7   
DIRV = 11
# Step pin from controller
STEPH = 33
STEPV = 32
# 0/1 used to signify clockwise or counterclockwise.
CW = 1
CCW = 0
GPIO.setwarnings(False)
# Setup pin layout on PI
GPIO.setmode(GPIO.BOARD)
# Establish Pins in software
GPIO.setup(DIRH, GPIO.OUT)
GPIO.setup(STEPH, GPIO.OUT)
GPIO.setup(DIRV, GPIO.OUT)
GPIO.setup(STEPV, GPIO.OUT)
# Set the first direction you want it to spin
GPIO.output(DIRH, CW)
GPIO.output(DIRV, CW)
class motorModules():
    #Move the motor during a number of steps
    def move_MotorX(stepsX,speed):
        # Set the first direction you want it to spin
        global directionX
        if stepsX < 0:
            directionX = CW
            stepsX = abs(stepsX)
        elif stepsX > 0:
            directionX = CCW
        else:
            stepsX = 0
        GPIO.output(DIRH, directionX)
        for x in range(stepsX // 20):
                GPIO.output(STEPH, GPIO.HIGH)
                sleep(speed)
                GPIO.output(STEPH, GPIO.LOW)
                sleep(speed)



    def move_MotorY(stepsY,speed):
        # Set the first direction you want it to spin
        global directionY
        if stepsY > 0:
            directionY = CW
        elif stepsY < 0:
            directionY = CCW
            stepsY = abs(stepsY)
        else:
            stepsY = 0
        print(stepsY)
        GPIO.output(DIRV, directionY)
        for x in range(stepsY // 20):
                GPIO.output(STEPV, GPIO.HIGH)
                sleep(speed)
                GPIO.output(STEPV, GPIO.LOW)
                sleep(speed)
