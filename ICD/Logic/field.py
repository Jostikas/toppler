from sortedcollections import ValueSortedDict
from house import House
import multiprocessing as mp
import Queue
import numpy as np
import cv2

D = 4 # Distance between a new house and any old house in centimeters for it to be considered a separate house.
H = 90  # Height and width of the field in cm, with (0,0) in the upper left-hand corner
W = 150
R_WEIGHT = 30  # Don't want the path to go there, like, at all
G_WEIGHT = -5  # To reduce the cost for approaching green houses near red ones, and knock over any on the way.
D_WEIGHT = 5  # Avoid if possible.

class FieldGUI(object):

    def __init__(self, id):
        """A GUI for a field instance. Will be passed to the field, which will know what to do with it.

        Will run in the GUI process, as the openCV GUI needs to be single-process. Data exchange with the field is via
        shared memory, and is initiated by the field.

        :param id: ID of the field. Used only for window naming
        """
        super(FieldGUI, self).__init__()
        self.id = id
        self.enabled = mp.Value('B', 0)  # Read by the Field instance to determine whether to bother updating the array.
        self.im_array = mp.Array('B', H*W*3)  # Field will update this memory, if the GUI is enabled.
        self.image = np.frombuffer(self.im_array.get_obj(), np.uint8, H * W * 3).reshape((H, W, 3))
        self.winname = 'Field {}'.format(id)

    def toggle(self):
        """Called by the main GUI to show/hide the field display."""
        if not self.enabled.value:
            # If not already enabled, create the windows and enable.
            # Doesn't need synchronization, as this is only called from the same process i.e. synchronously
            cv2.namedWindow(self.winname, True)
            cv2.moveWindow(self.winname, 600, self.id * 300)
            self.enabled.value = 1 
        else:
            self.enabled.value = 0
            cv2.destroyWindow(self.winname)

    def update(self):
        """Called by the main GUI loop to renew the display."""
        if self.enabled.value:
            cv2.imshow(self.winname, self.image)


class Field(mp.Process):
    """Represents the detected playing field and it's contents. Also handles path creation."""

    def __init__(self, id, car, gui):
        super(Field, self).__init__(target=self.dispatch, name='Field_{}_proc'.format(id))
        self.exit = mp.Event()
        self.id = id
        self.car = car
        self.gui = gui  # type: FieldGUI
        self.width = 150
        self.height = 90
        self.houses = dict()
        self.xes = ValueSortedDict()
        self.houseupdates = ValueSortedDict()  # Essentially a priority queue with more functionality
        self.update_idx = 0
        self.next_new_house = 0
        self.potentials = np.zeros((H, W), dtype = float)
        self.que =  mp.Queue(4)
        self.start()

    def stop(self):
        self.exit.set()

    def dispatch(self):
        while not self.exit.is_set():
            try:
                func, args, kwargs = self.que.get(timeout=1)
            except Queue.Empty:
                pass
            else:
                type(self).__dict__[func](self, *args, **kwargs)

    def reset(self):
        """Invalidate static info."""
        self.call('_reset')

    def _reset(self):
        self.houses.clear()
        self.houseupdates.clear()
        self.xes.clear()
        self.update_idx = 0
        self.next_new_house = 0

    def call(self, func, *args, **kwargs):
        self.que.put((func, args, kwargs))  # TODO: Change to put_nowait before the competition

    def update_houses(self, housearray):
        self.call('_update_houses', housearray)

    def _update_houses(self, housearray):
        self.update_idx += 1
        for ha in housearray:
            # Get an iterator over all houses that have center x coordinate within 5 cm of the input value.
            # Then see if one of the houses is close enough to be considered the same. If there are, update it's
            # location
            point = ha[2:4]
            to_change = None
            for oldhouse_idx in self.xes.irange_key(point[0] - D, point[0] + D):  # in cm.
                oldhouse = self.houses[oldhouse_idx]
                if -D < oldhouse.center.coords()[1] - point[1] < D and oldhouse.color == ha[1]:
                    to_change = oldhouse_idx
                    break
            if to_change is not None:
                house = self.houses[to_change]
                house.center.update(point)
            else:
                to_change = self.next_new_house
                self.next_new_house += 1
                house = House(point, ha[1])
            self.xes[to_change] = house.center.coords()[0]
            self.houses[to_change] = house
            self.houseupdates[to_change] = self.update_idx
        # if not self.update_idx % 5:
        self.clean_houses()
        self.update_potentials()
        if self.gui.enabled:
            self.draw_houses()

    def update_potentials(self):
        self.potentials = np.zeros((H, W))
        for house in self.houses.values(): # type: House
            if house.color == House.RED:
                weight = R_WEIGHT
            elif house.color == House.GREEN:
                weight = G_WEIGHT
            else:
                weight = D_WEIGHT
            x, y = house.center.coords()
            self.potentials[int(y - 2 + 0.5):int(y + 2 + 0.5), int(x - 2 + 0.5):int(x + 2 + 0.5)] = weight
            cv2.GaussianBlur(self.potentials, (31, 31), 4, self.potentials, borderType=cv2.BORDER_CONSTANT)
            np.maximum(self.potentials, 0, self.potentials)

    def draw_houses(self):
        frame = np.frombuffer(self.gui.im_array.get_obj(), np.uint8, H * W * 3).reshape((H, W, 3))
        frame[:] = cv2.cvtColor(cv2.convertScaleAbs(self.potentials, alpha=80), cv2.COLOR_GRAY2BGR)
        for house in self.houses.values():
            x, y = house.center.coords()
            xl, yl = max(x - 2, 0), max(y - 2, 0)
            xh, yh = min(x + 2, W - 1), min(y + 2, H - 1)
            if house.color == House.RED:
                col = (0, 0, 255)
            elif house.color == House.GREEN:
                col = (0, 255, 0)
                # frame[yl:yh, xl:xh] = col

    def clean_houses(self):
        """Clean up houses that haven't been updated in 5 or more turns."""
        indices = list(self.houseupdates.irange_key(None, self.update_idx - 5))
        for idx in indices:
            del self.xes[idx]
            del self.houses[idx]
            del self.houseupdates[idx]
