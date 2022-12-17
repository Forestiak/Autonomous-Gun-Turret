import RPi.GPIO as GPIO
from time import sleep


#Direction pin from controller
DIR = 7
#Step pin from controller
STEP = 33
#0/1 used to signify clockwise or counterclockwise.
CW = 0
CCW = 1

#Time it takes between steps
speed = 0.005

#Setup pin layout on PI
GPIO.setmode(GPIO.BOARD)

#Establish Pins in software
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

#Move the motor during a number of steps
def move_Motor(steps,speed,direction):
    # Set the first direction you want it to spin
    GPIO.output(DIR, direction)
    #Move the required number of steps
    for x in range(steps):
            GPIO.output(STEP, GPIO.HIGH)
            sleep(speed)
            GPIO.output(STEP, GPIO.LOW)
            sleep(speed)

#clean everything up
def clean_Motor():
    print("cleanup")
    GPIO.cleanup()

