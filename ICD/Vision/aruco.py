from cv2 import aruco
import numpy as np

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

arudict = aruco.custom_dictionary(8, 3)


def create_car_board(idx):
    return aruco.Board_create(objPoints, arudict, np.arange(0, 4) + idx * 4)
