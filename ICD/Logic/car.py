import multiprocessing as mp
import numpy as np
import cv2

class Car(object):
    def __init__(self, gui):
        super(Car, self).__init__()
        # Car location on the field
        self.x = mp.Value('d', 0)
        self.y = mp.Value('d', 0)
        # Car direction as normal vector on the field
        self.dx = mp.Value('d', 0)
        self.dy = mp.Value('d', 1)

        self.gui = gui

    def update_pos(self, loc, dir):
        """DynamicProcessor calls this to update the car's location and direction on a field."""
        self.x.value, self.y.value = loc
        self.dx.vale, self.dy.value = dir
        if self.gui.enabled:
            self.draw_car()

    def draw_car(self):
        """Draws the car on the field. Orange end is front."""
        l = 14  # Length and Width of car in cm
        w = 6
        frame = np.frombuffer(self.gui.im_array.get_obj(), np.uint8, H * W * 3).reshape((H, W, 3))
        x, y = self.x.value, self.y.value
        dx, dy = self.dx.value, self.dy.value
        cv2.line(frame,
                 (int(x + 0.5), int(y + 0.5)),
                 (int(x + l * dx * 0.6 + 0.5), int(y + l * dy * 0.6 + 0.5)),
                 (0, 127, 255),
                 thickness=w
                 )
        cv2.line(frame,
                 (int(x + 0.5), int(y + 0.5)),
                 (int(x - l * dx * 0.6 - 0.5), int(y + l * dy * 0.6 + 0.5)),
                 (255, 0, 0),
                 thickness=w
                 )

