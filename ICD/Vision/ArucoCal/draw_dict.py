import pickle
from cv2 import aruco
import cv2
import numpy as np

# arudict = aruco.Dictionary_get(pickle.load(f))
img1 = np.full((2970, 2100), 255, dtype=np.uint8)
arudict = aruco.Dictionary_create(8, 3)

side = 500


def draw_board(dict, start):
    img = np.full((1200, 1800), 255, dtype=np.uint8)
    cv2.rectangle(img, (1, 1), (1800, 1200), 220)
    cv2.line(img, (600, 0), (600, 1200), 220)
    cv2.line(img, (1200, 0), (1200, 1200), 220)
    aruco.drawMarker(dict, start + 0, side, img[50: 50 + side,
                                            900 - side // 2: 900 + side // 2])
    aruco.drawMarker(dict, start + 1, side, img[650: 650 + side,
                                            900 - side // 2: 900 + side // 2])
    aruco.drawMarker(dict, start + 2, side, img[600 - side // 2: 600 + side // 2,
                                            300 - side // 2: 300 + side // 2])
    aruco.drawMarker(dict, start + 3, side, img[600 - side // 2: 600 + side // 2,
                                            1500 - side // 2: 1500 + side // 2])
    return img


img = np.full((2970, 2100), 255, dtype=np.uint8)
img[200:200 + 1200, 150:150 + 1800] = draw_board(arudict, 0)
img[1570:1570 + 1200, 150:150 + 1800] = draw_board(arudict, 4)
cv2.imshow('Marker boards', cv2.resize(img, dsize=None, fx=0.3, fy=0.3))
cv2.waitKey(0)
col = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
col[img == 0] = [1, 2, 3]  # To force printer to use color toner for the field, for better black depth
cv2.imwrite('tagsBW.png', img)
cv2.imwrite('tagsColor.png', col)
