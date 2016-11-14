import cv2
import numpy as np
from functools import total_ordering

@total_ordering
class RAvg(object):
    """Keeps a running average"""

    def __init__(self, value, alpha=0.1):
        super(RAvg, self).__init__()
        self.value = value
        self.alpha = alpha
        self.counter = 0.  # Float because checking whether it should be updated or not will likely be more expensive
            # than just incrementing, and floats don't overflow.

    def __float__(self):
        return float(self.value)

    def __int__(self):
        return int(self.value)

    def __long__(self):
        return long(self.value)

    def __eq__(self, other):
        return self.value == other

    def __lt__(self, other):
        return self.value < other

    def __repr__(self):
        return 'RAvg({})'.format(self.value)



    def update(self, value):
        self.counter += 1
        alpha = max(1.0/self.counter, self.alpha)
        self.value = (1-alpha)*self.value + alpha*value

    def reset(self):
        self.counter = 0

class RAvgPoint(object):
    """Running average point in n-dimensional space"""

    def __init__(self, point, alpha=0.1):
        if alpha >= 1 or alpha <= 0:
            raise ValueError('alpha should be 0 < alpha < 1. Did you forget to put () around your point?')
        super(RAvgPoint, self).__init__()
        self.point = []
        self.point.extend(RAvg(coord, alpha) for coord in point)

    def reset(self):
        for coord in self.point:
            coord.reset()

    def coords(self):
        return tuple(float(val) for val in self.point)

    def update(self, point):
        for i, coord in enumerate(point):
            self.point[i].update(coord)

    def __getitem__(self, item):
        return self.point[item]

    @staticmethod
    def dist(p1, p2):
        a = np.array(p1.coords())
        b = np.array(p2.coords())
        return np.linalg.norm(b-a)

    @staticmethod
    def angle(p1, p2):
        a = np.array(p1.coords())
        if isinstance(p2, RAvgPoint):
            b = np.array(p2.coords())
        else:
            b = np.array(p2)
        return np.angle(complex(*(b-a)), True)

class Trig(object):
    pass

def draw_point(img, point, target=True, coords=True, color=(0,0,0), radius=5, thickness=1):
    point = tuple(int(ra.value) for ra in point)
    cv2.circle(img, point, radius, color, thickness)
    if target:
        cv2.line(img, (point[0] - 2*radius, point[1]), (point[0] + 2*radius, point[1]), color, thickness)
        cv2.line(img, (point[0], point[1] - 2*radius), (point[0], point[1] + 2*radius), color, thickness)
    if coords:
        cv2.putText(img, str(point), (point[0]+radius, point[1]+radius), cv2.FONT_HERSHEY_PLAIN, 0.5*thickness, color, thickness)

def largest_contour(contours):
    largest = 0
    idx = None
    for i, cont in enumerate(contours):
        area = cv2.contourArea(cont, oriented=False)
        if area > largest:
            largest = area
            idx = i
    return idx

def draw_min_rect(img, rect, **kwargs):
    points = cv2.boxPoints(rect)
    points = np.int0(points)
    cv2.drawContours(img, [points], 0, **kwargs)
    for i in range(4):
        # cv2.line(img, tuple(points[i-1]), tuple(points[i]), color=(255, 255, 255), thickness=3)
        p1 = points[i-1]
        xoffset = 10 if p1[0] < 320 else -80
        yoffset = 10 if p1[1] < 240 else -10
        cv2.putText(img,
                    '({:.1f}, {:.1f})'.format(*p1),
                    (int(p1[0])+xoffset, int(p1[1])+yoffset),
                    cv2.FONT_HERSHEY_PLAIN, 0.7, (0, 0, 0)
                    )

def cyclicInRange(src, lowerb, upperb):
    if lowerb[0] <= upperb[0]:
        return cv2.inRange(src, lowerb, upperb)
    else:
        lowermask = lowerb.copy()
        uppermask = upperb.copy()
        lowermask[0]=0
        uppermask[0] = upperb[0]
        mask1 = cv2.inRange(src, lowermask, uppermask)
        lowermask[0] = lowerb[0]
        uppermask[0] = 360
        mask2 = cv2.inRange(src, lowermask, uppermask)
        cv2.add(mask1, mask2, mask1)
        return mask1

