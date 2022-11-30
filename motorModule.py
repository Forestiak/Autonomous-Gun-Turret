import RPi.GPIO as GPIO
from time import sleep
import SSDMobileNetModule as sm

GPIO.cleanup()
# Direction pin from controller
DIR = 15
# Step pin from controller
STEP = 33
# 0/1 used to signify clockwise or counterclockwise.
CW = 1
CCW = 0

# Setup pin layout on PI
GPIO.setmode(GPIO.BOARD)

# Establish Pins in software
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

# Set the first direction you want it to spin
GPIO.output(DIR, CW)
class motorModules():
    def move(x):
        if x > 0:
            GPIO.output(DIR, 1)
        elif x < 0:
            GPIO.output(DIR,0)
        pass
        sleep(0.1)
        GPIO.output(STEP, GPIO.HIGH)
        sleep(.0005)
        GPIO.output(STEP, GPIO.LOW)
        sleep(.0005)


