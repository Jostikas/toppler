import ICDElectronics
import time

# Create a new electronics interface object
electronics = ICDElectronics.Interface("COM4")

#electronics.raw.electronics_ui_mainloop()

electronics.rotateRobot(1, -1000, 0)
# time.sleep(1)
#electronics.driveRobot(1, 1, 00)
#time.sleep(1.5)
#electronics.driveRobot(1, 0, 0)

# electronics.moveUpperRobot(1, 30)

print("--- END OF PROGRAM ---")




















# more functions to use:

# moving the upper robot sideways
# electronics.moveUpperRobot(0, -8) # move upper robot with ID 1 towards the starting area with a speed of 8
# electronics.moveUpperRobot(0, 0)  # stop the upper robot with ID 1 (sets the speed to 0)

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

# electronics.rotateRobot(1, 500)    # rotates the robot a few degrees one way
# electronics.rotateRobot(1, -500)   # rotates the robot a few degrees the other way