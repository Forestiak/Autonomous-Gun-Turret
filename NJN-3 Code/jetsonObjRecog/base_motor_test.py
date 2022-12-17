import RPi.GPIO as GPIO
from time import sleep

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

try:
    # Run forever.
    while True:

        """Change Direction: Changing direction requires time to switch. The
        time is dictated by the stepper motor and controller. """
    
        # Esablish the direction you want to go
        GPIO.output(DIR, CW)
        sleep(1.0)
        # Run for 200 steps. This will change based on how you set you controller
        for x in range(200):
            # Set one coil winding to high
            GPIO.output(STEP, GPIO.HIGH)
            # Allow it to get there.
            sleep(.0005)  # Dictates how fast stepper motor will run
            # Set coil winding to low
            GPIO.output(STEP, GPIO.LOW)
            sleep(.0005)  # Dictates how fast stepper motor will run

        """Change Direction: Changing direction requires time to switch. The
        time is dictated by the stepper motor and controller. """
        sleep(1.0)
        GPIO.output(DIR, CCW)
        for x in range(200):
            GPIO.output(STEP, GPIO.HIGH)
            sleep(.0005)
            GPIO.output(STEP, GPIO.LOW)
            sleep(.0005)

# Once finished clean everything up
except KeyboardInterrupt:
    print("cleanup")
    GPIO.cleanup()
