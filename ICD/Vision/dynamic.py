from cv2 import aruco


class DynamicProcessor(object):
    """Processes frames for car detection and potential other moving stuff."""

    def __init__(self, car, board, shape):
        super(DynamicProcessor, self).__init__()
        self.car = car
        self.board = board

    def process_frame(sel):
        pass
