import RPi.GPIO as GPIO
from time import sleep
import SSDMobileNetModule as sm
direction = 0
GPIO.cleanup()
# Direction pin from controller
DIRH = 7   
DIRV = 16
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
    def move_MotorX(steps,speed):
        # Set the first direction you want it to spin
        global direction
        if steps < 0:
            direction = CW
            steps = abs(steps)
        elif steps > 0:
            direction = CCW
        else:
            steps = 0
        GPIO.output(DIRH, direction)
        for x in range(steps // 20):
                GPIO.output(STEPH, GPIO.HIGH)
                sleep(speed)
                GPIO.output(STEPH, GPIO.LOW)
                sleep(speed)
