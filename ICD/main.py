from __future__ import print_function
from Vision.vision import FrameProcessor
from Logic.field import Field
from Logic.car import Car
import cv2
import numpy as np
from Vision.common import RAvg
from time import time

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FPS, 60)

cv2.namedWindow('Main', True)
mainpic = np.full((300,300), 255, dtype=np.uint8)
maintext = "Hotkeys:\n" \
           "1: Toggle field 1 calibration\n" \
           "2: Toggle field 2 calibration (N/I)\n" \
           "r: Toggle RAW display\n" \
           "f: Toggle field display (N/I)\n" \
           "h: Toggle Hue display\n" \
           "s: Toggle Sat display\n" \
           "v: Toggle Val display\n" \
           "Esc: Quit"
for n, line in enumerate(maintext.splitlines()):
    cv2.putText(mainpic, line, (10,(n+1)*20), cv2.FONT_HERSHEY_PLAIN, 1, 0)
cv2.imshow('Main', mainpic)

field = Field(0)
car = Car()
fproc = FrameProcessor((480, 640, 3), field, car)

dispatch = {'1': fproc.static.toggle_GUI,
            'r': lambda : fproc.toggle_screen('raw'),
            'h': lambda : fproc.toggle_screen('H'),
            's': lambda : fproc.toggle_screen('S'),
            'v': lambda : fproc.toggle_screen('V'),
            }

# import cProfile
# def profile():
#     frames = []
#     starttime = time()
#     st = starttime
#     for i in range(100):
#         ret, frame = cam.read()
#         proc.process_frame(frame)
#         cv2.waitKey(1)
#         t = time()
#         frames.append(1/(t - starttime))
#         starttime = t
#     print(np.array(frames).astype(np.uint8))
#     print(100/(time()-st))

# for i in range(10):
#     ret, frame = cam.read()
# cProfile.run('profile()',sort='tottime')

while True:
    flag, frame = cam.read()
    if not flag:
        print('Failed to capture frame.')
        break
    fproc.process_frame(frame)
    # print(fproc.static.get_corners())
    key = cv2.waitKey(1) & 0xff
    if key == 27:
        break
    elif key != -1: # as 255 is returned on first frame
        dispatch.get(chr(key), lambda : None)()



print('Exit-key: {}'.format(key & 0xff))
cam.release()
cv2.destroyAllWindows()
