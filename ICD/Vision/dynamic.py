from cv2 import aruco
import cam_params


class DynamicProcessor(object):
    """Processes frames for car detection and potential other moving stuff."""

    def __init__(self, car, board, shape):
        super(DynamicProcessor, self).__init__()
        self.car = car
        self.board = board

    def car_to_field(self, rvec, tvec):
        """Transform the car's location and direction to field coordinates, shift to field plane.

        :param rvec: Rotation vector (as output from aruco.estimatePoseBoard)
        :param tvec: Translation vector (as output from aruco.estimatePoseBoard)
        :return: loc, dir - Location in field coordinates, direction vector (normalized) in field coords.
        """

        pass

    def process_frame(self, frame):
        cam_mat = cam_params.logitech_matrix
        cam_dist = cam_params.logitech_dist_coeffs
        corners, ids, rejected = aruco.detectMarkers(frame, arudict)
        corners, ids, rejected, recovered = aruco.refineDetectedMarkers(frame, self.board, corners, ids, rejected,
                                                                        cam_mat, cam_dist)
        N, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, cam_mat, cam_dist)
        if N:
            self.car.update_pos(*self.car_to_field(rvec, tvec))

