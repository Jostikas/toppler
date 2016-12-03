import cv2
from Logic.robot import Robot
from Electronics.ICDElectronics import Interface
from time import sleep
from common import FRAME_H, FRAME_W, FPS

cam0 = cv2.VideoCapture(1)
cam1 = cv2.VideoCapture(0)

cam0 = cv2.VideoCapture(1)
cam1 = cv2.VideoCapture(0)
cam0.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cam0.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cam1.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cam1.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)

cam0.set(cv2.CAP_PROP_FPS, FPS)
cam1.set(cv2.CAP_PROP_FPS, FPS)

electronics = Interface('port')

robot0 = Robot(0, cam0, electronics)
robot1 = Robot(1, cam1, electronics)

while True:
    sleep(1)
    if not (robot0.isAlive() or robot1.isAlive()):
        break
print('Done.')
