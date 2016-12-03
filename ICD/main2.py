import cv2
from Logic.robot import Robot
from Electronics.ICDElectronics import Interface
from time import sleep

cam0 = cv2.VideoCapture(1)
cam1 = cv2.VideoCapture(0)
electronics = Interface('port')

robot0 = Robot(0, cam0, electronics)
robot1 = Robot(1, cam1, electronics)

while True:
    sleep(1)
    if not (robot0.isAlive() or robot1.isAlive()):
        break
print('Done.')
