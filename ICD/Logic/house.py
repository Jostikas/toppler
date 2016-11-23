from ICD.common import RAvgPoint
class House(object):

    block = (40, 40, 40)
    GREEN = 0
    RED = 1
    DEBRIS = 2  # maybe unused.

    def __init__(self, point, color, width=1, depth=1, height=1, rot=0):
        """Defines a house.

        :arg color: One of GREEN, RED or DEBRIS.
        :arg width: Width of house in blocks.
        :arg depth: Depth of house in blocks.
        :arg height: Height of house in blocks.
        :arg rot: Rotation of house in degrees.
        """
        super(House, self).__init__()
        self.center = RAvgPoint(point)
        self.width = width
        self.depth = depth
        self.height = height
        self.color = color
        self.rot = rot