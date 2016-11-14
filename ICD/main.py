from Vision.vision import FrameProcessor
from Logic.field import Field
from Logic.car import Car
import cv2
import numpy as np

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FPS, 60)

field = Field(0)
car = Car()
proc = FrameProcessor((480,640,3), field, car)

dispatch = {'1': proc.static.toggle_GUI}

cv2.namedWindow('H', True)
cv2.namedWindow('S', True)
cv2.namedWindow('V', True)
cv2.moveWindow('H', 30+500, 1080 / 3 * 0)
cv2.moveWindow('S', 30+500, 1080 / 3 * 1)
cv2.moveWindow('V', 30+500, 1080 / 3 * 2)

while  True:
    flag, frame = cam.read()
    if not flag:
        print('Failed to capture frame.')
        break
    proc.process_frame(frame)
    cv2.imshow('blah', frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
    cv2.imshow('H', hsv[:,:,0])
    cv2.imshow('S', hsv[:, :, 1])
    cv2.imshow('V', hsv[:, :, 2])
    key = cv2.waitKey(1) & 0xff
    if key == 27:
        break
    elif key != -1 and key != 255:
        dispatch.get(chr(key), lambda : None)()



print('Exit-key: {}'.format(key & 0xff))
cam.release()
cv2.destroyAllWindows()
