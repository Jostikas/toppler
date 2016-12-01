from static import StaticProcessor
from dynamic import DynamicProcessor
import cv2
from aruco import create_car_board

# default states
defaultshow = {'raw': False,
               'H': False,
               'S': False,
               'V': False}
windowlocations = {'raw': (1920 / 3 * 1, 30),
               'H': (1920 / 3 * 0, 530),
               'S': (1920 / 3 * 1, 530),
               'V': (1920 / 3 * 2, 530)}


class FrameProcessor(object):
    """Process frames. yay. Maybe should thread this, really not sure.
    If possible, I'd rather not, then camera can keep timestep for other stuff too."""

    def __init__(self, shape, field, car, idx):
        super(FrameProcessor, self).__init__()
        self.static = StaticProcessor(field, shape, avg=15)
        self.dynamic = DynamicProcessor(car, shape, create_car_board(idx))
        self.show = {'raw': False, 'H': False, 'S': False, 'V': False}
        self.show_default_screens()

    def show_default_screens(self):
        for key, value in defaultshow.items():
            if value and not self.show[key]:
                self.toggle_screen(key)

    def toggle_screen(self, screen):
        if self.show[screen]:
            cv2.destroyWindow(screen)
            self.show[screen] = False
        else:
            cv2.namedWindow(screen, True)
            cv2.moveWindow(screen, *windowlocations[screen])
            self.show[screen] = True

    def update_screens(self, frame):
        if self.show['raw']:
            cv2.imshow('raw', frame)
        h, s, v = self.show['H'], self.show['S'], self.show['V']
        if h or s or v:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
            if h:
                cv2.imshow('H', hsv[:,:,0])
            if s:
                cv2.imshow('S', hsv[:, :, 1])
            if v:
                cv2.imshow('V', hsv[:, :, 2])

    def process_frame(self, frame):
        """Dispatches frame for processing.

        :arg frame: BGR image"""
        self.static.accumulate_static(frame)
        self.update_screens(frame)

