import numpy as np
import cv2


while True:
    cv2.imshow('blah', np.zeros((100,100)))
    key = cv2.waitKey(0)
    if key != -1:
        print chr(key & 0xff)
    if key & 0xff == 27:
        break
