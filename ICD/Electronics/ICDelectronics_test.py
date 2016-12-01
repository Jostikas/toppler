import ICDElectronics
import time

# Create a new electronics interface object
electronics = ICDElectronics.Interface("COM11")

# move upper robot with ID 1 towards the starting area with a speed of 8
electronics.moveUpperRobot(1, -8)

print("Robot liigub ...")
time.sleep(1.4)

# stop the upper robot with ID 1 (sets the speed to 0)
electronics.moveUpperRobot(1, 0)

# more functions to use:

# moving the crane up and down
# electronics.moveCrane(10)     # moves the crane slowly upwards
# electronics.moveCrane(-10)    # moves the crane slowly downwards
# electronics.moveCrane(0)      # stops the crane from moving

# melting the fishing line
# electronics.meltFishingLine(100)    # tries to melt the fishing line for some time
# electronics.meltFishingLine(0)      # stops melting (could be used as a safety function if something goes really bad)

# moving the lower robots:
# electronics.driveRobot(0, 10)   # drives the car with an ID of 0 forward
# electronics.driveRobot(1, -10)  # drives the car with an ID of 1 backward
# electronics.driveRobot(0, 0)    # stops the car with an ID of 0

# electronics.rotateRobot(500)    # rotates the robot a few degrees one way
# electronics.rotateRobot(-500)   # rotates the robot a few degrees the other way

print("Robot peatatud ja programm ka lõppes")
