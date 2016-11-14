import numpy as np
from ast import literal_eval

import cv2

from common import RAvgPoint, largest_contour, draw_point

default_settings = {'min_intersect_area': 200,
                    'red_clip': [0, 169, 134, 17, 255, 255],
                    'max_area': 3800,
                    'green_clip': [26, 90, 25, 55, 255, 185],
                    'target': 3}
GREEN = 0
RED = 1
calibrate_more = True



class StaticSettingsGUI(object):

    def __init__(self, static_proc):
        super(StaticSettingsGUI, self).__init__()
        id = static_proc.field.id
        settings = static_proc.settings
        disp_name = 'Static {}'.format(id)
        settings_name = disp_name + ' settings'
        self.static_proc = static_proc
        self.disp_name = disp_name
        self.settings_name = settings_name
        cv2.namedWindow(disp_name, True)
        cv2.moveWindow(disp_name, 1920 / 3 * (id % 3), 30)
        cv2.namedWindow(settings_name, True)
        cv2.moveWindow(settings_name, 1920 / 3 * (id % 3), 30+700)
        self.color_selector = 0  # 0 green, 1 red
        self.ignore_trackbar_change = False
        self.img = np.empty(self.static_proc.shape, dtype=np.uint8)

        cv2.createTrackbar('RG', self.settings_name, self.color_selector, 1, self.color_selector_callback)
        cv2.createTrackbar('Hue_min', self.settings_name, settings['green_clip'][0], 360, self.update_cliprange)
        cv2.createTrackbar('Hue_max', self.settings_name, settings['green_clip'][3], 360, self.update_cliprange)
        cv2.createTrackbar('Sat_min', self.settings_name, settings['green_clip'][1], 255, self.update_cliprange)
        cv2.createTrackbar('Sat_max', self.settings_name, settings['green_clip'][4], 255, self.update_cliprange)
        cv2.createTrackbar('Val_min', self.settings_name, settings['green_clip'][2], 255, self.update_cliprange)
        cv2.createTrackbar('Val_max', self.settings_name, settings['green_clip'][5], 255, self.update_cliprange)
        cv2.createTrackbar('ROI Thresh', self.settings_name, settings['green_ROI'], 255, self.update_ROI_thresh)

    def store_settings(self):
        with open(self.static_proc.settingsfile, 'w') as f:
            print(self.static_proc.settings)
            f.write(repr(self.static_proc.settings))

    def color_selector_callback(self, val):
        self.color_selector = val
        sliders = ['Hue_min', 'Sat_min', 'Val_min', 'Hue_max', 'Sat_max', 'Val_max']
        self.ignore_trackbar_change = True
        settings = self.static_proc.settings
        for pos, name in enumerate(sliders):
            cv2.setTrackbarPos(name, self.settings_name,
                               settings['red_clip'][pos] if val else settings['green_clip'][pos])
        self.ignore_trackbar_change = False

    def update_cliprange(self, _):
        if self.ignore_trackbar_change:  # Changing from green to red settings.
            return
        sliders = ['Hue_min', 'Sat_min', 'Val_min', 'Hue_max', 'Sat_max', 'Val_max']
        for val, key in enumerate(sliders):
            if self.color_selector:  # G = 0, R = 1
                self.static_proc.settings['red_clip'][val] = cv2.getTrackbarPos(key, self.settings_name)
            else:
                self.static_proc.settings['green_clip'][val] = cv2.getTrackbarPos(key, self.settings_name)

    def update_ROI_thresh(self, val):
        self.static_proc.settings['green_ROI'] = val

    def update(self):
        cv2.imshow(self.disp_name, self.img)

    def close(self):
        self.store_settings()
        cv2.destroyWindow(self.disp_name)
        cv2.destroyWindow(self.settings_name)


class StaticProcessor(object):

    def __init__(self, field, shape=(480,640,3), settingsfile='Vision/static_settings.txt', avg=10):
        """Processes the frames to detect (quasi)static features in the scene.

        :arg field: a Field instance to apply the data to.
        :arg shape: shape of the frames for averaging
        :arg avg: Averaging factor. The frame rate is decimated by the same amount.
        """
        super(StaticProcessor, self).__init__()
        self.field = field
        self.avg = avg
        self.settingsfile = settingsfile
        self.counter = 0  # Triggers processing when it reaches avg
        self.shape = shape
        self.accumulator = np.zeros(shape, dtype=np.float32)
        self.corners = [RAvgPoint((40, 40)),
                        RAvgPoint((600, 40)),
                        RAvgPoint((600, 600)),
                        RAvgPoint((40, 600))
                        ]
        # The center is duplicate information, but one that is available freely.
        # Also, it's needed for ROI processing.
        self.center = RAvgPoint((320, 240))
        self.read_settings(settingsfile)
        self.calibrate = False
        self.toggle_GUI()  #Uncomment if you want the calibration screen to immediately appear.


    def toggle_GUI(self):
        if self.calibrate:
            self.calibrate = False
            self.GUI.close()
            del self.GUI
        else:
            self.calibrate = True
            self.GUI = StaticSettingsGUI(self)

    def read_settings(self, settingsfile):
        with open(settingsfile, 'r') as f:
            try:
                settings = literal_eval(f.read())
                # Check that we have all necessary settings.
                assert len(settings['red_clip']) == 6
                assert len(settings['green_clip']) == 6
                settings['target']  # Statement checks whether settings were read correctly.
                settings['min_intersect_area']
                settings['max_area']

            except IOError:
                print('Static settings file not found. Using default.')
                settings = default_settings
            except (KeyError, AssertionError):
                print('Failed to read settings. Using default.')
                settings = default_settings
            self.settings = settings

    def get_field_rect(self):
        c = self.center.coords()
        w = RAvgPoint.dist(self.corners[0], self.corners[1])
        h = RAvgPoint.dist(self.corners[1], self.corners[2])
        a = RAvgPoint.angle(self.corners[0], self.corners[1])
        return c, (w, h), a

    def reset(self):
        """Reset averaging."""
        self.counter = 0
        self.accumulator = np.zeros(self.shape, np.float32)
        for point in self.corners:
            point[0].reset()
            point[1].reset()
        self.center[0].reset()
        self.center[1].reset()

    def update_center(self, point):
        self.center.update(point)
        if self.calibrate:
            draw_point(self.GUI.img, self.center)

    def update_corners(self, points):
        loc = ((0, 3), (1, 2))  # Used to match input points to corners
        for i in range(4):
            j = loc[points[i][0] > self.center[0]][points[i][1] > self.center[1]]
            self.corners[j].update(points[i])
        if self.calibrate:  # Display the rectangle
            points = np.int0(points)
            cv2.drawContours(self.GUI.img, [points], 0, (255, 255, 0), 2)
            for point in self.corners:
                draw_point(self.GUI.img, point, color= (0, 200, 200), thickness=2)

    def accumulate_static(self, frame):
        """Accumulate frames for averaging for static analysis of the playing field.

        Only calculates the result once every avg frames. If reset is True, will purge old results.

        :arg frame: A frame of type CV_8UC3 and size self.shape
        :arg reset: Reset averaging
        """

        self.counter += 1
        cv2.accumulate(frame, self.accumulator)

        if not self.counter % self.avg:  # if avg samples have been collected
            self.process_static(self.accumulator)
            self.counter = 0
            self.accumulator = np.zeros(frame.shape, dtype=np.float32)

    def process_static(self, frame):
        """Process the averaged image, update field.

        :arg frame: The averaged frame, of type CV_32F.
        :arg reset: Whether to reset the running average of field corners."""
        frame = (frame // self.avg)  # type: np.ndarray
        frame = frame.astype(np.uint8)
        if self.calibrate:  # Store input for debugging
            self.GUI.img = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.cvtColor(frame, cv2.COLOR_BGR2HSV, frame)

        self.find_field(gray)
        self.find_houses(frame, GREEN)
        self.find_houses(frame, RED)
        if self.calibrate:
            self.GUI.update()

    def find_field(self, gray):
        img = cv2.medianBlur(gray, 7)
        f_mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 7, 5)
        cv2.morphologyEx(f_mask, cv2.MORPH_OPEN, None, f_mask, iterations=2)

        if self.calibrate:  # Contour search is destructive, so have to show it here or copy stuff.
            self.GUI.img[f_mask > 0] = [255, 255, 255]

        _, contours, hier = cv2.findContours(f_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        field_box = cv2.minAreaRect(contours[largest_contour(contours)])
        self.update_center(field_box[0])
        self.update_corners(cv2.boxPoints(field_box))


    def find_houses(self, hsv, green_red):
        """Segment scene by color to find red or green houses.

        :arg hsv: HSV image of the scene.
        :arg green_red: True finds red houses, False the green ones."""
        ROIs = self.find_house_ROIs(hsv, green_red)
        filt_labels = self.filter_ROIs(ROIs)
        if filt_labels: # If any ROIs passed inspection,
            houses = self.ROI_analysis(hsv, ROIs, filt_labels, green_red)


    def find_house_ROIs(self, hsv, green_red):
        """Find and label general location of houses by color segmentation

                :arg hsv: HSV image to search from.
                :arg clip: Range of color to segment.
                :return N, labels, stats, centroids: See output of connectedComponentsWithStats. labels is a CV_16U array.
                """
        clip = self.settings['red_clip' if green_red else 'green_clip']
        img = cv2.medianBlur(hsv, 5)
        mask = cv2.inRange(img, np.array(clip[0:3]), np.array(clip[3:6]))
        cv2.erode(mask, None, mask, iterations=3)
        cv2.medianBlur(mask, 3, mask)
        cv2.dilate(mask, None, mask, None, 3)
        ROIs = cv2.connectedComponentsWithStats(mask, ltype=cv2.CV_16U)
        if self.calibrate:
            self.GUI.img[mask > 0] = (0, 0, 255) if green_red else (0, 255, 0)  # output the mask
        return ROIs

    def filter_ROIs(self, ROIs):
        """Filter ROIs based on location and area.

        :arg ROIs: Output from cv2.connectedComponentsWithStats()
        :return filtered: List of labels that passed filtering."""
        filtered = []
        N, labels, stats, centroids = ROIs
        field_rect = self.get_field_rect()
        for idx in range(1, N):  # 0 is the background
            #First test, because it's free: If area is too large, exclude it.
            if stats[idx, cv2.CC_STAT_AREA] > self.settings['max_area']:
                continue

            # Second test: exclude areas that don't intersect with the playing field
            c = tuple(centroids[idx])
            w = stats[idx, cv2.CC_STAT_WIDTH]
            h = stats[idx, cv2.CC_STAT_HEIGHT]
            a = 0
            label_rect = (c, (w,h), a)
            intersect, points = cv2.rotatedRectangleIntersection(field_rect, label_rect)
            if intersect == cv2.INTERSECT_NONE:
                continue

            # Third test: if the area of intersection is too small, exclude it.
            points = cv2.convexHull(points)
            if cv2.contourArea(points) < self.settings['min_intersect_area']:
                continue

            # Fourth test: if the area of intersection is very narrow (usually on the border), exclude it.
            _, (width, height), _ = cv2.minAreaRect(points)
            r = width / height
            if not (0.25 < r < 4):
                continue

            filtered.append(idx)
            if self.calibrate:
                points = np.int0(points)
                cv2.drawContours(self.GUI.img, [points], 0, color = (255, 0, 0), thickness=2)
                cv2.putText(self.GUI.img,
                            str(stats[idx, cv2.CC_STAT_AREA]),
                            (int(c[0]), int(c[1])),
                            cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0)
                            )
        return filtered

    def _get_ROI_transform(self, stats, idx):
        # Calculate the necessary transform for ROI isolation.
        x1, y1 = stats[idx, cv2.CC_STAT_LEFT], stats[idx, cv2.CC_STAT_TOP]
        x2, y2 = x1 + stats[idx, cv2.CC_STAT_WIDTH], y1 + stats[idx, cv2.CC_STAT_HEIGHT]
        xc, yc = (x2 - x1) // 2, (y2 - y1) // 2  # Got center of bounding box
        dsize = int(1.5 * max(stats[idx, cv2.CC_STAT_WIDTH:cv2.CC_STAT_HEIGHT]))  # Should fit all rotations
        # rot = cv2.getRotationMatrix2D((xc, yc), RAvgPoint.angle(self.center, centroids[idx]), 1)
        rot = np.eye(3)
        rot[0:2, 0:3] = cv2.getRotationMatrix2D((dsize / 2, dsize / 2),
                                                RAvgPoint.angle(self.center, (x1+xc, y1+yc)) + 90, 1)
        tran = np.eye(3)
        tran[0:2, 2] = ((dsize / 2 - xc), (dsize / 2 - yc))
        mat = (np.dot(rot, tran))[0:2]
        window = x1, x2, y1, y2
        return mat, window, dsize

    def ROI_analysis(self, hsv, ROIs, filt_labels, green_red):
        """Analyse each ROI, determine center of base, rotation, height-if-possible.

        :arg hsv: HSV image of the scene.
        :arg ROIs: output from find_house_ROIs.
        :arg filt_labels: labels that were determined to potentially be houses.
        :arg green_red: Whether we're looking for a green or a red house.
        """
        # Debug window management
        if calibrate_more:
            pos = {'sigma': 5,
                   'ksize': 21,
                   'theta': 0,
                   'lambd': 0,
                   'gamma': 0,
                   'psi': 0
                   }
            cv2.namedWindow('gabor', True)
            cv2.createTrackbar('ksize', 'gabor', 1, 100, lambda x: pos.__setitem__('ksize', x))
            cv2.createTrackbar('sigma', 'gabor', 1, 100, lambda x: pos.__setitem__('ksize', x))
            cv2.createTrackbar('theta', 'gabor', 1, 100, lambda x: pos.__setitem__('ksize', x))
            cv2.createTrackbar('lambd', 'gabor', 1, 100, lambda x: pos.__setitem__('ksize', x))
            cv2.createTrackbar('gamma', 'gabor', 1, 100, lambda x: pos.__setitem__('ksize', x))
            cv2.createTrackbar('psi', 'gabor', 1, 100, lambda x: pos.__setitem__('ksize', x))
            img = np.zeros((640, 640, 3), dtype=np.uint8)
            x = 0
            y = 0
            y_max = 0

        N, labels, stats, centroids = ROIs
        for idx in filt_labels:
            # Find the transformation window
            mat, (x1, x2, y1, y2), dsize = self._get_ROI_transform(stats, idx)
            invmat = cv2.invertAffineTransform(mat) # needed to relate the extracted features into world-space.

            snip = cv2.warpAffine(hsv[y1:y2, x1:x2], mat, (dsize, dsize))
            # snip = cv2.warpAffine(hsv[labels == idx], mat, (dsize, dsize))
            snip = cv2.cvtColor(snip, cv2.COLOR_HSV2BGR_FULL)
            # snip = cv2.cvtColor(snip, cv2.COLOR_BGR2GRAY)
            # if green_red == GREEN:
            #     snip = snip[:,:,2]
            #     thresh, snip = cv2.threshold(snip, self.settings['green_ROI'], 255, cv2.THRESH_BINARY)
#            cv2.getGaborKernel(5, )
            if calibrate_more:
                y_max = max(y+dsize, y_max)
                if x + dsize > 640:
                    x = 0
                    y = y_max
                img[y:y+dsize, x:x+dsize] = snip
                x += dsize
        if calibrate_more:
            cv2.imshow('{}'.format(('red' if green_red else 'green')), img)