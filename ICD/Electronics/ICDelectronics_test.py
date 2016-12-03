import ICDElectronics
import time

# Create a new electronics interface object
electronics = ICDElectronics.Interface("COM11")

def test():
    print("Well hello world")

electronics.raw.setStartCallback(test)

'''
# move upper robot with ID 1 towards the starting area with a speed of 8
electronics.moveUpperRobot(1, -8)
print("Robot liigub ...")
time.sleep(1.4)
# stop the upper robot with ID 1 (sets the speed to 0)
electronics.moveUpperRobot(1, 0)
print("Robot peatatud ja programm ka lõppes")
'''