import cv2

from numpy import mean, binary_repr, zeros
from numpy.random import randint
import numpy as np

from ar_markers.hamming.coding import encode, HAMMINGCODE_MARKER_POSITIONS

MARKER_SIZE = 7
ZOOM_RATIO = 50


class HammingMarker(object):
    def __init__(self, id, size=1, contours=None, rvec=None, tvec=None):
        self.id = id
        self.contours = contours
        self.size = size
        self.rvec = rvec
        self.tvec = tvec

    def __repr__(self):
        return '<Marker id={} center={}>'.format(self.id, self.center)


    @property
    def center(self):
        if self.contours is None:
            return None
        center_array = mean(self.contours, axis=0).flatten()
        return (int(center_array[0]), int(center_array[1]))


    @property
    def origin(self):
        if self.contours is None:
            return None
        return (int(self.contours[0][0][0]), int(self.contours[0][0][1]))


    def generate_image(self):
        img = zeros((MARKER_SIZE, MARKER_SIZE))
        img[1, 1] = 255  # set the orientation marker
        for index, val in enumerate(self.hamming_code):
            coords = HAMMINGCODE_MARKER_POSITIONS[index]
            if val == '1':
                val = 255
            img[coords[0], coords[1]] = int(val)
        # return img
        output_img = zeros((MARKER_SIZE*ZOOM_RATIO, MARKER_SIZE*ZOOM_RATIO))
        cv2.resize(img, dsize=((MARKER_SIZE*ZOOM_RATIO, MARKER_SIZE*ZOOM_RATIO)), dst=output_img, interpolation=cv2.INTER_NEAREST)
        return output_img


    def draw_contour(self, img, color=(0, 255, 0), linewidth=2):
        if self.contours != None:
            cv2.drawContours(img, [self.contours], -1, color, linewidth)


    def draw_origin(self, img, color=(0, 0, 255), radius=2, linewidth=3):
        if self.origin != None:
            cv2.circle(img, self.origin, radius=radius, color=color, thickness=linewidth)


    def draw_local_frame(self, img, camK, linewidth=3):
        # only display when pose exist
        if self.size == None or self.rvec == None or self.tvec == None:
            return
        # get plot obj points
        obj_points = np.array([(0,0,0), (0.8*self.size,0,0), (0,0.8*self.size,0), (0,0,0.8*self.size)], \
            dtype='float32')
        # convert to img points
        img_points = cv2.projectPoints(obj_points, self.rvec, self.tvec, camK, np.array([0,0,0,0], \
            dtype='float32'))[0]
        cv2.line(img, tuple(img_points[0][0]), tuple(img_points[1][0]), (0,0,255),thickness=linewidth)
        cv2.line(img, tuple(img_points[0][0]), tuple(img_points[2][0]), (0,255,0),thickness=linewidth)
        cv2.line(img, tuple(img_points[0][0]), tuple(img_points[3][0]), (255,0,0),thickness=linewidth)


    def highlite_marker(self, img, contour_color=(0, 255, 255), text_color=(255, 255, 0), \
            linewidth=2, draw_frame=False, camK=None):
        self.draw_contour(img, color=contour_color, linewidth=linewidth)
        if draw_frame:
            if camK == None:
                raise ValueError("Require camK to plot local frame.")
            self.draw_local_frame(img, camK)
        else:
            self.draw_origin(img)
        cv2.putText(img, str(self.id), self.center, cv2.FONT_HERSHEY_DUPLEX, 1, text_color)


    @classmethod
    def generate(cls):
        return HammingMarker(id=randint(4096))


    @property
    def id_as_binary(self):
        return binary_repr(self.id, width=12)


    @property
    def hamming_code(self):
        return encode(self.id_as_binary)
