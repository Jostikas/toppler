from sortedcontainers import SortedDict
from house import House
import multiprocessing as mp
import numpy as np
import cv2

D = 4 # Distance between a new house and any old house in centimeters for it to be considered a separate house.
H = 90  # Height and width of the field in cm, with (0,0) in the upper left-hand corner
W = 150

class FieldGUI(object):

    def __init__(self, id):
        """A GUI for a field instance. Will be passed to the field, which will know what to do with it.

        Will run in the GUI process, as the openCV GUI needs to be single-process. Data exchange with the field is via
        shared memory, and is initiated by the field.

        :param id: ID of the field. Used only for window naming
        """
        super(FieldGUI, self).__init__()
        self.enabled = mp.Value('B', 0)  # Read by the Field instance to determine whether to bother updating the array.
        self.im_array = mp.Array('B', H*W*3)  # Field will update this memory, if the GUI is enabled.
        self.image = np.frombuffer(self.im_array, np.uint8, H*W*3).reshape((H,W,3))
        self.winname = 'Field {}'.format(id)

    def toggle(self):
        """Called by the main GUI to show/hide the field display."""
        if not self.enabled.value:
            # If not already enabled, create the windows and enable.
            # Doesn't need synchronization, as this is only called from the same process i.e. synchronously
            cv2.namedWindow(self.winname, True)
            cv2.moveWindow(self.winname, 600, id*300)
            self.enabled.value = 1 
        else:
            self.enabled.value = 0
            cv2.destroyWindow(self.winname)

    def update(self):
        """Called by the main GUI loop to renew the display."""
        if self.enabled:
            cv2.imshow(self.winname, self.image)


class Field(mp.Process):
    """Represents the detected playing field and it's contents."""

    def __init__(self, id, gui):
        super(Field, self).__init__(target=self.dispatch, name='Field_{}_proc'.format(id), daemon=True)
        self.exit = mp.Event()
        self.id = id
        self.gui = gui
        self.width = 150
        self.height = 90
        self.houses = SortedDict(lambda x: self.houses[x].center.coords[0])
        self.houseupdates = SortedDict(lambda x: self.houseupdates[x]) # Essentially a priority queue with more functionality
        self.update_idx = 0
        self.que =  mp.Queue(4)
        self.start()

    def dispatch(self):
        while not self.exit.is_set():
            func, args, kwargs = self.que.get()
            func(*args, **kwargs)

    def reset(self):
        """Invalidate static info."""
        raise NotImplementedError

    def call(self, func, *args, **kwargs):
        self.que.put((func, args, kwargs))  # TODO: Change to put_nowait before the competition

    def update_houses(self, housearray):
        self.call(self._update_houses, housearray)

    def _update_houses(self, housearray):
        self.update_idx += 1
        for ha in housearray:
            # Get an iterator over all houses that have center x coordinate within 5 cm of the input value.
            # Then see if one of the houses is close enough to be considered the same. If there are, update it's
            # location
            point = ha[2]
            for oldhouse_idx in self.houses.irange_key(point[0] - D, point[0] + D):  # in cm.
                oldhouse = self.houses[oldhouse_idx]
                if -D < oldhouse.center.coords[1] - point[1] < D:
                    # So there is a house nearby enough that instead of making a new one, we should update the old.
                    oldhouse.center.update(point)
                    self.houseupdates[oldhouse_idx] = self.update_idx
                    self.houses.update({oldhouse_idx: oldhouse})
                    break
            else:
                # No house was found in a 2D by 2D square around the input house. So we need to create a new one.
            color = ha[1]
            newhouse = House(point, color)
            newhouse_idx = max(self.houses.keys()) + 1
            self.houses.update({newhouse_idx: newhouse})
            self.houseupdates.update({newhouse_idx: self.update_idx})



