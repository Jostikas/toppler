from multiprocessing

class Field(object):
    """Represents the detected playing field and it's contents.
    """

    def __init__(self, id):
        super(Field, self).__init__()
        self.id = id
        self.width = 150
        self.height = 90
        self.houses = dict()
        self.que =

    def reset(self):
        """Invalidate static info."""
        pass

    def
