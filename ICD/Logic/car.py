import multiprocessing as mp

class Car(object):
    def __init__(self):
        super(Car, self).__init__()
        self.x = mp.Value('d')
        self.y = mp.Value('d')
