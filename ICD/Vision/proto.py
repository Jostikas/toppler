import cv2
import numpy as np
from ast import literal_eval

capProps = dict(W=cv2.CAP_PROP_FRAME_WIDTH,
                H=cv2.CAP_PROP_FRAME_HEIGHT,
                FPS=cv2.CAP_PROP_FPS,
                FORMAT=cv2.CAP_PROP_FORMAT,
                MODE=cv2.CAP_PROP_MODE,
                BRI=cv2.CAP_PROP_BRIGHTNESS,
                CON=cv2.CAP_PROP_CONTRAST,
                SAT=cv2.CAP_PROP_SATURATION,
                HUE=cv2.CAP_PROP_HUE,
                GAIN=cv2.CAP_PROP_GAIN,
                EXP=cv2.CAP_PROP_EXPOSURE,
                AE=cv2.CAP_PROP_AUTO_EXPOSURE)

cam = cv2.VideoCapture(0)
cam.set(capProps['FPS'], 60)




set_winname = 'Settings'
cv2.namedWindow(set_winname)
in_winname = 'Raw'
med_winname = 'In-process'
out_winname = 'Out'
settingsfile = "settings.txt"
# kernel = np.asarray([[0, 255, 255, 255, 0],
#                    [255, 255, 255, 255, 255],
#                    [255, 255, 255, 255, 255],
#                    [255,255, 255, 255, 255],
#                    [0,255, 255, 255, 0]], dtype=np.uint8)
kernel = np.asarray([[0, 255, 0],
                   [0, 255, 0],
                   [255, 255, 255],
                   [0, 255, 0],
                   [0, 255, 0]], dtype=np.uint8)

cliprange = [1880, 540, 0, 390, 2550, 2550] # Default values
settings = dict(cliprange=cliprange, target=60)

with open(settingsfile, 'r') as f:
    settings = literal_eval(f.read())
    cliprange = settings['cliprange']

def store_settings():
    with open(settingsfile, 'w') as f:
        print(settings)
        f.write(repr(settings))

def update_cliprange(_):
    sliders = ['Hue_min', 'Sat_min', 'Val_min', 'Hue_max', 'Sat_max', 'Val_max']
    for val, key in enumerate(sliders):
        cliprange[val] = cv2.getTrackbarPos(key, set_winname)
    settings['cliprange'] = cliprange

def update_target(val):
    settings['target'] = val

a = cv2.createTrackbar('Hue_min', set_winname, cliprange[0], 3600, update_cliprange)
cv2.createTrackbar('Hue_max', set_winname, cliprange[3], 3600, update_cliprange)
cv2.createTrackbar('Sat_min', set_winname, cliprange[1], 2550, update_cliprange)
cv2.createTrackbar('Sat_max', set_winname, cliprange[4], 2550, update_cliprange)
cv2.createTrackbar('Val_min', set_winname, cliprange[2], 2550, update_cliprange)
cv2.createTrackbar('Val_max', set_winname, cliprange[5], 2550, update_cliprange)
cv2.createTrackbar('Target', set_winname, settings['target'], 2550, update_target)

def cyclicInRange(src, lowerb, upperb):
    if lowerb[0] <= upperb[0]:
        return cv2.inRange(src, lowerb, upperb)
    else:
        lowermask = lowerb.copy()
        uppermask = upperb.copy()
        lowermask[0]=0
        uppermask[0] = upperb[0]
        mask1 = cv2.inRange(src, lowermask, uppermask)
        lowermask[0] = lowerb[0]
        uppermask[0] = 3600
        mask2 = cv2.inRange(src, lowermask, uppermask)
        cv2.add(mask1, mask2, mask1)
        return mask1


while True:
    ret, raw = cam.read()
    if not ret:
        print("Failed to capture image.")
        break
    #cv2.GaussianBlur(im, (7, 7), 5, im)
    #cv2.imshow(in_winname, im)
    hsv = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV_FULL)
    #im[:, :, 2] = cv2.equalizeHist(im[:,:,2])
    mask = cyclicInRange(hsv, np.array(cliprange[0:3]), np.array(cliprange[3:6]))
    cv2.erode(mask, kernel, mask, iterations=2)
    mask = cv2.medianBlur(mask, 9)
    #mask = cv2.medianBlur(mask, 7)
    #mask = cv2.medianBlur(mask, 7)
    cv2.dilate(mask, kernel, mask, iterations=2)
    #cv2.imshow(med_winname, mask)
    cv2.cvtColor(raw, cv2.COLOR_BGR2HSV_FULL, hsv)
    hsv[mask > 0] = [settings['target'], 255, 200]
    # hsv = np.ones(hsv.shape, hsv.dtype)
    # hsv[:,:] = [settings['target'],255,255]
    out = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR_FULL)
    cv2.imshow(out_winname, out)

    if cv2.waitKey(1) != -1:
        print("Interrupted.")
        break
store_settings()
cam.release()
cv2.destroyAllWindows()