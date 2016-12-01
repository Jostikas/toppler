import cv2
from cv2 import aruco
import cam_params
import numpy as np
from ICD.common import *
from aruco import ht


class DynamicProcessor(object):
    """Processes frames for car detection and potential other moving stuff."""

    def __init__(self, master, car, board, shape):
        super(DynamicProcessor, self).__init__()
        self.master = master
        self.car = car
        self.board = board

    def car_to_field(self, rvec, tvec):
        """Transform the car's location and direction to field coordinates, shift to field plane.

        :param rvec: Rotation vector (as output from aruco.estimatePoseBoard)
        :param tvec: Translation vector (as output from aruco.estimatePoseBoard)
        :return: loc, dir - Location in field coordinates, direction vector (normalized) in field coords.
        """
        cam_mat = cam_params.logitech_matrix
        cam_dist = cam_params.logitech_dist_coeffs
        mat = np.frombuffer(self.field.perspective_matrix.get_obj())
        if mat.any():  # If the perspective matrix has already been updated by StaticProcessor
            obj_points = [[0,0,0], [0,1,0]]
            pic_points, _ = cv2.projectPoints(obj_points, rvec, tvec, cam_mat, cam_dist)
            cx, cy = self.master.center.coords()
            lens, angles = cv2.cartToPolar(cx - pic_points[:,0], cy - pic_points[:,1])
            lens *= (1 - 0.085/H_CAM_MM/1000)
            pic_points = np.array(cv2.polarToCart(lens, angles)).T
            field_points = cv2.perspectiveTransform(pic_points, mat)
            loc = field_points[0]
            dir = field_points[1] - field_points[0]
            return loc, dir
        else:
            return None, None


        pass

    def process_frame(self, frame):
        cam_mat = cam_params.logitech_matrix
        cam_dist = cam_params.logitech_dist_coeffs
        corners, ids, rejected = aruco.detectMarkers(frame, arudict)
        corners, ids, rejected, recovered = aruco.refineDetectedMarkers(frame, self.board, corners, ids, rejected,
                                                                        cam_mat, cam_dist)
        N, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, cam_mat, cam_dist)
        if N:
            loc, dir = self.car_to_field(rvec, tvec)
            if loc is not None:
                self.car.update_pos(loc, dir)

