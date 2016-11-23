from cv2 import aruco
import cv2
import numpy as np
from ICD.Vision.common import *

# Board critical dimensions
sd = 0.0472  # marker side length
ht = 0.00  # top of the robot
hs = 0.075 - 0.0795  # Side marker top edge height
sp = 0.0095  # spacing of top markers
wr = 0.061  # Width of robot

objPoints = np.array([[[[-sd / 2., sp / 2. + sd, ht]], [[sd / 2., sp / 2. + sd, ht]], [[sd / 2., sp / 2., ht]],
                       [[-sd / 2., sp / 2., ht]]],
                      [[[-sd / 2., -sp / 2, ht]], [[sd / 2., -sp / 2., ht]], [[sd / 2., -(sp / 2. + sd), ht]],
                       [[-sd / 2., -(sp / 2. + sd), ht]]],
                      [[[-wr / 2., sd / 2., hs - sd]], [[-wr / 2., sd / 2., hs]], [[-wr / 2., -sd / 2., hs]],
                       [[-wr / 2., -sd / 2., hs - sd]]],
                      [[[wr / 2., sd / 2., hs]], [[wr / 2., sd / 2., hs - sd]], [[wr / 2., -sd / 2., hs - sd]],
                       [[wr / 2., -sd / 2., hs]]]], dtype=np.float32)

# objPoints = np.array([[[[-sd/2., sp/2.+sd, ht]], [[sd/2., sp/2.+sd, ht]], [[sd/2., sp/2., ht]], [[-sd/2., sp/2., ht]]],
#     [[[-sd/2., -sp/2, ht]], [[sd/2., -sp/2., ht]], [[sd/2., -(sp/2.+sd), ht]], [[-sd/2., -(sp/2.+sd), ht]]]], dtype=np.float32)

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cam.set(cv2.CAP_PROP_FPS, FPS)
arudict = aruco.custom_dictionary(8, 3)
camera_matrix = np.array([5.4149050361000627e+02, 0., 3.2304389583118632e+02, 0.,
                          5.4125590709427399e+02, 2.3403508925960239e+02, 0., 0., 1.])
camera_matrix = camera_matrix.reshape((3, 3))
dist_coeffs = np.array([-1.0437158943177836e-01, 1.1861521365727017e-01,
                        1.6787284723004615e-03, -7.2814039128852074e-04,
                        -2.1159177335098589e-02])

car_board = aruco.Board_create(objPoints, arudict, np.arange(0, 4))

while True:
    ret, frame = cam.read()
    corners, ids, rejected = aruco.detectMarkers(frame, arudict)
    corners, ids, rejected, recovered = aruco.refineDetectedMarkers(frame, car_board, corners, ids, rejected,
                                                                    camera_matrix, dist_coeffs)
    aruco.drawDetectedMarkers(frame, corners, ids)
    # rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners, sd, camera_matrix, dist_coeffs)
    N, rvec, tvec = aruco.estimatePoseBoard(corners, ids, car_board, camera_matrix, dist_coeffs)
    if N:
        # for i in range(len(ids)):
        #     aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.4)
        aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.4)
        cv2.putText(frame, 'N = {}'.format(N), (20, 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255))
    cv2.imshow('blah', frame)
    if cv2.waitKey(1) != -1:
        break
