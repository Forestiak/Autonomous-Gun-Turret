import RPi.GPIO as GPIO
from time import sleep
import SSDMobileNetModule as sm

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
    def move(x):
    #    try:
        if x < 0:
            GPIO.output(DIRH, CW)
            sleep(0.005)
        elif x > 0:
            GPIO.output(DIRH, CCW)
            sleep(0.005)
        else: pass
        for x in range(50):
            GPIO.output(STEPH, GPIO.HIGH)
            sleep(0.0005)
            GPIO.output(STEPH, GPIO.LOW)
            sleep(0.0005)
    #    except KeyboardInterrupt:
    #        print("cleanup")
    #        GPIO.cleanup()
    # def movev(y):
    #     if y > 0:
    #         GPIO.output(DIRV, 1)
    #     elif y < 0:
    #         GPIO.output(DIRV, 0)
    #     else: pass
    #     sleep(0.1)
    #     GPIO.output(STEPV, GPIO.HIGH)
    #     sleep(.00005)
    #     GPIO.output(STEPV, GPIO.LOW)
    #     sleep(.00005)
