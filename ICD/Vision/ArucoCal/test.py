from cv2 import aruco
import cv2
import numpy as np
from ICD.Vision.common import *
from ICD.common import putTextMultiline

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
logitech_cam_matrix = np.array([6.6419013281274511e+02, 0., 3.3006505461506646e+02, 0.,
                                6.6587003740196826e+02, 2.3030870519881907e+02, 0., 0., 1.])
logitech_cam_matrix = logitech_cam_matrix.reshape((3, 3))
logitech_dist_coeffs = np.array([2.0806786957819065e-01, -3.6839605435678208e-01,
                                 -1.2078578161331370e-02, 6.4294407773653481e-03,
                                 4.2723310854154123e-01])

car_board0 = aruco.Board_create(objPoints, arudict, np.arange(0, 4))
car_board1 = aruco.Board_create(objPoints, arudict, np.arange(4, 8))

while True:
    ret, frame = cam.read()
    corners, ids, rejected = aruco.detectMarkers(frame, arudict)
    corners, ids, rejected, recovered = aruco.refineDetectedMarkers(frame, car_board0, corners, ids, rejected,
                                                                    logitech_cam_matrix, logitech_dist_coeffs)

    corners, ids, rejected, recovered = aruco.refineDetectedMarkers(frame, car_board1, corners, ids, rejected,
                                                                    logitech_cam_matrix, logitech_dist_coeffs)
    aruco.drawDetectedMarkers(frame, corners, ids)
    # rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners, sd, camera_matrix, dist_coeffs)
    N0, rvec0, tvec0 = aruco.estimatePoseBoard(corners, ids, car_board0, logitech_cam_matrix, logitech_dist_coeffs)
    N1, rvec1, tvec1 = aruco.estimatePoseBoard(corners, ids, car_board1, logitech_cam_matrix, logitech_dist_coeffs)
    if N0:
        # for i in range(len(ids)):
        #     aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.4)
        mat, jacob = cv2.Rodrigues(rvec0)
        dash = np.full((400, 600), 255, dtype=np.uint8)
        putTextMultiline(dash, repr(mat), (20, 20))
        putTextMultiline(dash, repr(tvec0), (400, 200))
        putTextMultiline(dash, repr(rvec0), (20, 200))
        cv2.imshow('Dash', dash)

        aruco.drawAxis(frame, logitech_cam_matrix, logitech_dist_coeffs, rvec0, tvec0, 0.4)
    if N1:
        aruco.drawAxis(frame, logitech_cam_matrix, logitech_dist_coeffs, rvec1, tvec1, 0.4)
    cv2.imshow('blah', frame)
    if cv2.waitKey(1) != -1:
        break
