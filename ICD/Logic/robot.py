import cv2
import numpy as np
from ICD.Vision.vision import FrameProcessor
from field import Field
from car import Car
from ICD.gui import GUI
from ICD.common import FRAME_H, FRAME_W

import threading as th

# CONSTANTS - STATENAME_CONSTANT = VALUE
READY_WAIT_FOR_SIGNAL = False

BLIND_TIME_0 = 2.  # Time for mothership 0 to drive from start position without checking
BLIND_TIME_1 = 5.  # Time for mothership 1 to drive from start position without checking
BLIND_SPEED = 0.  # Speed of blind drive

TAXI_CHECK_SPEED = 0.  # Speed of drive while searching for field
TAXI_CHECK_NO_OF_CHECKS = 5  # Number of frames that field must be true to be accepted
TAXI_CHECK_THRESHHOLD = 0.05  # Target position accuracy in frame heights


class Robot(th.Thread):
    """Handles top-level management for one robot (mothership-car pair)"""

    def __init__(self, idx, cam, electronics, gui):
        super(Robot, self).__init__(target=self.main, name="Robot {}".format(idx))
        self.idx = idx
        self.cam = cam
        self.elec = electronics
        self.gui = gui
        self.car = Car(idx)
        self.field = Field(idx)
        self.car = Car(gui)
        self.vision = FrameProcessor((FRAME_H, FRAME_W, 3), self.field, self.car, idx)
        self.game_start = th.Event()
        self.game_stop = th.Event()
        self.state = 0
        self.state_lookup = dict(enumerate([
            self.ready,
            self.blind,
            self.taxi,
            self.drop,
            self.play
        ]))

        self.start()

    def main(self):
        """"""
        while self.state in self.state_lookup:
            if self.game_stop.isSet():
                break
            flag, frame = self.cam.read()
            increment_state = False
            while not increment_state:
                increment_state = self.state_lookup[self.state](frame)
                self.gui.update(self.state, frame)
            self.state += 1
            increment_state = False
        print('Robot {} was stopped externally.'.format(self.idx))

    def ready(self, _):
        """State waits for start signal."""
        done = self.game_start.isSet()
        return done

    def blind(self, _):
        """State taxis the motherships blindly."""
        done = False
        if self.idx == 0:
            done = True
        else:

        return done

    def taxi(self, frame):
        """State handles transporting the robot to the field using feedback."""
        done = False
        field_pos = self.vision.field_pos(frame)

        if abs(field_pos) < TAXI_CHECK_THRESHHOLD:
            done = True
        return done

    def drop(self, frame):
        """State handles dropping the car."""
        if not self.idx:
            return True  # Robot 0 doesn't need to do this

    def play(self, frame):
        """State handles gameplay"""
        done = False
        return done
