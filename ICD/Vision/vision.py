from static import StaticProcessor
from dynamic import DynamicProcessor

class FrameProcessor(object):
    """Process frames. yay. Maybe should thread this, really not sure.
    If possible, I'd rather not, then camera can keep timestep for other stuff too."""
    def __init__(self, shape, field, car):
        super(FrameProcessor, self).__init__()
        self.static = StaticProcessor(field, shape)
        #self.dynamic = DynamicProcessor(car, shape)

    def process_frame(self, frame):
        """Dispatches frame for processing.

        :arg frame: BGR image"""
        self.static.accumulate_static(frame)

