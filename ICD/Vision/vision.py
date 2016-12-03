from static import StaticProcessor
from dynamic import DynamicProcessor
import cv2
from aruco import create_car_board
from ICD.common import FRAME_H, FRAME_W, RAvgPoint, draw_houghline
import multiprocessing as mp
import numpy as np
from ast import literal_eval

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
        self.idx = idx
        self.field = field
        self.center = RAvgPoint((FRAME_W // 2, FRAME_H // 2))
        self.static = StaticProcessor(self, field, shape, avg=15)
        self.dynamic = DynamicProcessor(self, car, create_car_board(idx), shape)
        self.perspective_matrix = mp.Array('d', 9)
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
        self.dynamic.process_frame(frame)
        self.update_screens(frame)

    def field_pos(self, frame):
        """Calculate the direction of the field center relative to the camera

        Used during taxiing.
        """
        settingsfile = 'Vision/static_settings.txt'
        with open(settingsfile, 'r') as f:
            settings = literal_eval(f.read())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)[:,:,2]  # Value part of HSV
        gray = cv2.copyMakeBorder(gray, 30, 30, 0, 0, borderType=cv2.BORDER_REPLICATE)
        thresh, f_mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        f_mask = cv2.Laplacian(f_mask, cv2.CV_8U, f_mask, 5)
        lines = cv2.HoughLines(f_mask, self.settings['rho'] + 1, self.settings['theta'] * np.pi / 180 + 0.01,
                               self.settings['Htres'], srn=5, stn=5)
        # Filter the lines: we only want the horizontal ones.
        toplines = []
        bottomlines = []
        topweights = []
        bottomweights = []
        for line in lines:
            # Ignore lines that are not horizontal
            if (0.4 * np.pi) < line[1] < (0.6 * np.pi):
                continue
            weight = self.get_houghline_direction(gray, line)
            if -0.05 < weight < 0.05 and self.idx == 0:
                continue  # Likely the start line on field 1
            if weight > 0:
                toplines.append(line)
                topweights.append(line)
            else:
                bottomlines.append(line)
                bottomweights.append(weight)
        top = None
        bottom = None
        if toplines:
            top = np.average(toplines, 0, topweights)
            a = np.cos(top[1])
            b = np.sin(top[1])
            x0 = a * top[1]
            y0 = b * top[1]
            xs = (x0 - FRAME_W / 2) / b
            topy = b * top[0] + FRAME_W/2
        if bottomlines:
            bottom = np.average(bottomlines, 0, bottomweights)
            a = np.cos(bottom[1])
            b = np.sin(bottom[1])
            x0 = a * bottom[1]
            y0 = b * bottom[1]
            xs = (x0 - FRAME_W/2) / b
            bottomy = y0 + xs * a
        if top is not None and bottom is not None:
            shift = 0.5 * (1 - (bottomy + topy) * 1. / FRAME_H)
        elif top is not None:
            shift = 0.05 - topy * 1./ FRAME_H
        elif bottom is not None:
            shift = 0.95 - bottomy * 1. / FRAME_H
        else:
            shift = 1  # Assume that the field is further ahead.

    def get_houghline_direction(self, frame, line):
        """Calculate the direction of the hough line.

        :return: direction < 0 for bottom lines, > 0 for top lines"""
        rho, theta = np.ravel(line)
        mask = np.zeros(frame.shape, dtype=np.int32)
        subline = (line[0]+20, line[1])
        superline = (line[0]-20, line[1])
        draw_houghline(mask, subline, 1,)
        draw_houghline(mask, superline, -1)
        total = np.sum(mask * frame)
        weight = total*1. /  180000
        return weight
