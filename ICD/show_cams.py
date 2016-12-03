import cv2
from common import FRAME_H, FRAME_W, FPS

cam0 = cv2.VideoCapture(0)
cam1 = cv2.VideoCapture(1)

cam0.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cam0.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cam1.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cam1.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)

cam0.set(cv2.CAP_PROP_FPS, FPS)
cam1.set(cv2.CAP_PROP_FPS, FPS)

cv2.namedWindow('CAM0', True)
cv2.namedWindow('CAM1', True)

cv2.moveWindow('CAM0', 0, 0)
cv2.moveWindow('CAM1', 850, 0)

while True:
    ret, frame0 = cam0.read()
    ret, frame1 = cam1.read()
    cv2.imshow('CAM0', frame0)
    cv2.imshow('CAM1', frame1)
    if cv2.waitKey(1) & 0xff == 27:
        break
