import cv2
import cam_params
import numpy as np
from ICD.common import *
from aruco import arudict


class DynamicProcessor(object):
    """Processes frames for car detection and potential other moving stuff."""

    def __init__(self, master, car, board, shape):
        super(DynamicProcessor, self).__init__()
        self.master = master
        self.car = car
        self.board = board
        self.detparam = cv2.aruco.DetectorParameters_create()
        self.detparam.minMarkerPerimeterRate = (120 / pixres) / FRAME_W
        self.detparam.maxMarkerPerimeterRate = (240 / pixres) / FRAME_W
        self.detparam.minCornerDistanceRate = 0.15
        self.detparam.minMarkerDistanceRate = 0.03
        self.detparam.polygonalApproxAccuracyRate = 0.05
        # self.detparam.doCornerRefinement = True
        self.detparam.cornerRefinementWinSize = 2
        self.detparam.perspectiveRemovePixelPerCell = 10

    def car_to_field(self, rvec, tvec):
        """Transform the car's location and direction to field coordinates, shift to field plane.

        :param rvec: Rotation vector (as output from aruco.estimatePoseBoard)
        :param tvec: Translation vector (as output from aruco.estimatePoseBoard)
        :return: loc, dir - Location in field coordinates, direction vector (normalized) in field coords.
        """
        cam_mat = cam_params.logitech_matrix
        cam_dist = cam_params.logitech_dist_coeffs
        mat = np.frombuffer(self.master.perspective_matrix.get_obj()).reshape((3, 3))
        if mat.any():  # If the perspective matrix has already been updated by StaticProcessor
            obj_points = np.array([[0, 0, 0], [0, 1, 0]], np.float32)
            pic_points, _ = cv2.projectPoints(obj_points, rvec, tvec, cam_mat, cam_dist)
            cx, cy = self.master.center.coords()
            lens, angles = cv2.cartToPolar(cx - pic_points[:, 0, 0], cy - pic_points[:, 0, 1])
            lens *= (1 - 85. / H_CAM_MM)
            pic_points = np.array(cv2.polarToCart(lens, angles)).T
            pic_points = pic_points[0, :, None]  # Cause OpenCV is ...
            pic_points = np.array((cx, cy)) - pic_points
            field_points = cv2.perspectiveTransform(pic_points, mat)
            loc = field_points[0, 0]
            dir = field_points[1, 0] - field_points[0, 0]
            cv2.normalize(dir, dir, 70 * pixres, norm_type=2)
            return loc, dir
        else:
            return None, None

    def process_frame(self, frame):
        cam_mat = cam_params.logitech_matrix
        cam_dist = cam_params.logitech_dist_coeffs
        cv2.setNumThreads(
            0)  # Python multiprocessing and OpenCV multithreading have trust issues. See https://github.com/opencv/opencv/issues/5150
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, arudict, parameters=self.detparam)
        corners, ids, rejected, recovered = cv2.aruco.refineDetectedMarkers(frame, self.board, corners, ids, rejected,
                                                                            cam_mat, cam_dist,
                                                                            errorCorrectionRate=-1,
                                                                            parameters=self.detparam)
        N, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, self.board, cam_mat, cam_dist)
        cv2.setNumThreads(-1)
        if N:
            cv2.aruco.drawAxis(frame, cam_mat, cam_dist, rvec, tvec, 0.4)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            loc, dir = self.car_to_field(rvec, tvec)
            if loc is not None:
                self.car.update_pos(loc, dir)
        pass
