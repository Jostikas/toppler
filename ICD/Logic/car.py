import multiprocessing as mp

class Car(object):
    def __init__(self):
        super(Car, self).__init__()
        # Car location on the field
        self.x = mp.Value('d', 0)
        self.y = mp.Value('d', 0)
        # Car direction as normal vector on the field
        self.dx = mp.Value('d', 0)
        self.dy = mp.Value('d', 1)

    def update_pos(self, loc, dir):
        """DynamicProcessor calls this to update the car's location and direction on a field."""
        self.x.value, self.y.value = loc
        self.dx.vale, self.dy.value = dir
