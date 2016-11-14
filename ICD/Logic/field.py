class Field(object):
    """Represents the detected playing field and it's contents.
    """

    def __init__(self, id):
        super(Field, self).__init__()
        self.id = id
        self.width = 1500
        self.height = 900
