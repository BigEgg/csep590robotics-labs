#!/usr/bin/env python3

''' Get a raw frame from camera and display in OpenCV
'''

import cv2
import cozmo
import numpy as np
import math

from numpy.linalg import inv

from ar_markers.hamming.detect import detect_markers

from utils import *

#obtain marker distance and orientation
def cvt_marker_measurements(ar_markers):

    marker2d_list = [];

    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0,0,1], [0,-1,0], [1,0,0]])
        R_2_2p = np.matrix([[0,-1,0], [0,0,-1], [1,0,0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        print('\n', R_2p_1p)
        yaw = -math.atan2(R_2p_1p[2,0], R_2p_1p[0,0])
        #print('\nyaw', yaw)
        x, y = m.tvec[2][0], -m.tvec[0][0]
        #print('x =', x, 'y =', y)
        
        # remove any duplate markers
        dup_thresh = 2.0
        find_dup = False
        for m2d in marker2d_list:
            if grid_distance(m2d[0], m2d[1], x, y) < dup_thresh:
                find_dup = True
                break
        if not find_dup:
            marker2d_list.append((x,y,yaw))

    return marker2d_list


def display_opencv(robot: cozmo.robot.Robot):

    # params
    camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')
    marker_size = 4.8

    # start streaming
    robot.camera.image_stream_enabled = True

    while True:

        latest_image = robot.world.latest_image

        if latest_image is not None:

            # convert pil to opencv
            open_cv_image = np.array(latest_image.raw_image)

            #detect markers
            markers = detect_markers(open_cv_image, marker_size, camK)

            for marker in markers:
                marker.highlite_marker(open_cv_image, draw_frame=True, camK=camK)
                #print("ID =", marker.id);
                #print(marker.contours);

            cv2.imshow("Markers", open_cv_image)
            cv2.waitKey(1)

            m2d_list = cvt_marker_measurements(markers)
            print(m2d_list)


cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
cozmo.run_program(display_opencv, use_viewer=False)
