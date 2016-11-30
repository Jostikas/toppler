from __future__ import print_function
from Vision.vision import FrameProcessor
from Vision.common import FRAME_H, FRAME_W, FPS
from Logic.field import Field, FieldGUI
from Logic.car import Car
import cv2
import numpy as np
from Vision.common import RAvg
from time import time

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)

cam.set(cv2.CAP_PROP_FPS, FPS)

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

field0_gui = FieldGUI(0)
field = Field(0, field0_gui)
car = Car()
fproc = FrameProcessor((FRAME_H, FRAME_W, 3), field, car)

dispatch = {'1': fproc.static.toggle_GUI,
            'f': lambda: field0_gui.toggle(),
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
#         fproc.process_frame(frame)
#         cv2.waitKey(1)
#         t = time()
#         frames.append(1/(t - starttime))
#         starttime = t
#     print(np.array(frames).astype(np.uint8))
#     print(100/(time()-st))
#
# for i in range(10):
#     ret, frame = cam.read()
# profile()

while True:
    flag, frame = cam.read()
    if not flag:
        print('Failed to capture frame.')
        break
    fproc.process_frame(frame)
    field0_gui.update()
    # print(fproc.static.get_corners())
    key = cv2.waitKey(1) & 0xff
    if key == 27:
        break
    elif key != -1: # as 255 is returned on first frame
        dispatch.get(chr(key), lambda : None)()



print('Exit-key: {}'.format(key & 0xff))
field.stop()
field.join()
cam.release()
cv2.destroyAllWindows()
