from __future__ import print_function
from Vision.vision import FrameProcessor
from common import FRAME_H, FRAME_W, FPS
from Logic.field import Field, FieldGUI
from Logic.car import Car
import cv2
import numpy as np
from Vision.common import RAvg
from time import time

# import start

cam0 = cv2.VideoCapture(1)
cam1 = cv2.VideoCapture(0)
cam0.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cam0.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)

cam0.set(cv2.CAP_PROP_FPS, FPS)

cv2.namedWindow('Main', True)
mainpic = np.full((300,300), 255, dtype=np.uint8)
maintext = "Hotkeys:\n" \
           "1: Toggle field 1 calibration\n" \
           "2: Toggle field 2 calibration (N/I)\n" \
           "r: Toggle RAW display\n" \
           "f: Toggle field 1 display\n" \
           "g: Toggle field 2 display\n" \
           "h: Toggle Hue display\n" \
           "s: Toggle Sat display\n" \
           "v: Toggle Val display\n" \
           "Esc: Quit"

for n, line in enumerate(maintext.splitlines()):
    cv2.putText(mainpic, line, (10,(n+1)*20), cv2.FONT_HERSHEY_PLAIN, 1, 0)
cv2.imshow('Main', mainpic)

field0_gui = FieldGUI(0)
car0 = Car(field0_gui)
field0 = Field(0, car0, field0_gui)
fproc0 = FrameProcessor((FRAME_H, FRAME_W, 3), field0, car0, 0)

field1_gui = FieldGUI(1)
car1 = Car(field1_gui)
field1 = Field(1, car1, field1_gui)
fproc1 = FrameProcessor((FRAME_H, FRAME_W, 3), field1, car1, 1)

dispatch = {'1': fproc0.static.toggle_GUI,
            'f': lambda: field0_gui.toggle(),
            'r': lambda: fproc0.toggle_screen('raw'),
            'h': lambda: fproc0.toggle_screen('H'),
            's': lambda: fproc0.toggle_screen('S'),
            'v': lambda: fproc0.toggle_screen('V'),
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

# start.taxi(cam0, cam1, field1, fproc1)  # field is needed to find a free spot for the car.


while True:
    flag, frame = cam0.read()
    if not flag:
        print('Failed to capture frame.')
        break
    fproc0.process_frame(frame)
    field0_gui.update()
    # print(fproc.static.get_corners())
    key = cv2.waitKey(1) & 0xff
    if key == 27:
        break
    elif key != -1: # as 255 is returned on first frame
        dispatch.get(chr(key), lambda : None)()



print('Exit-key: {}'.format(key & 0xff))
field0.stop()
field0.join()
cam0.release()
cv2.destroyAllWindows()
