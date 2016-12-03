import cv2
from Logic.robot import Robot
from Electronics.ICDElectronics import Interface
from time import sleep
from common import FRAME_H, FRAME_W, FPS, CAM0, CAM1, PORT

from Logic.field import FieldGUI

cam0 = cv2.VideoCapture(CAM0)
cam1 = cv2.VideoCapture(CAM1)

cam0.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cam0.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cam1.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cam1.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)

cam0.set(cv2.CAP_PROP_FPS, FPS)
cam1.set(cv2.CAP_PROP_FPS, FPS)

electronics = Interface(PORT)

field0_gui = FieldGUI(0)
robot0 = Robot(0, cam0, electronics, field0_gui)
robot1 = Robot(1, cam1, electronics, field0_gui)


def game_start():
    robot0.game_start.set()
    robot1.game_start.set()


def game_stop():
    robot0.game_stop.set()
    robot1.game_stop.set()


electronics.setStartCallback(game_start)
electronics.setStopCallback(game_stop)

while True:
    sleep(1)
    if not (robot0.isAlive() or robot1.isAlive()):
        break
print('Done.')
