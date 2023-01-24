class motorModules():
    #Move the motor during a number of steps
    def move_MotorX(stepsX,speed):
        # Set the first direction you want it to spin
        global directionX
        # This is dependant on which way the A and B Ports on the motor driver are connected
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
