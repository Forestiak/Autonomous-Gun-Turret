import time

# Export pin 12 as an output
with open("/sys/class/gpio/export", "w") as export_file:
    export_file.write("12")

# Set pin 12 as an output
with open("/sys/class/gpio/gpio12/direction", "w") as direction_file:
    direction_file.write("out")

# Turn on laser pointer
with open("/sys/class/gpio/gpio12/value", "w") as value_file:
    value_file.write("1")

# Keep script running, laser pointer will stay on
while True:
    time.sleep(1)

# Unexport pin 12
with open("/sys/class/gpio/unexport", "w") as unexport_file:
    unexport_file.write("12")
