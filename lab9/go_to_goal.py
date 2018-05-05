#!/usr/bin/env python3

''' Get a raw frame from camera and display in OpenCV
By press space, save the image from 001.bmp to ...
'''

import cv2
import cozmo
import numpy as np
from numpy.linalg import inv
import threading
import time

from ar_markers.hamming.detect import detect_markers

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *


# camera params
camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')

#marker size in inches
marker_size = 3.5

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
gui = GUIWindow(grid)



async def image_processing(robot):

    global camK, marker_size

    event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # convert camera image to opencv format
    opencv_image = np.asarray(event.image)

    # detect markers
    markers = detect_markers(opencv_image, marker_size, camK)

    # show markers
    for marker in markers:
        marker.highlite_marker(opencv_image, draw_frame=True, camK=camK)
        #print("ID =", marker.id);
        #print(marker.contours);
    cv2.imshow("Markers", opencv_image)

    return markers

#calculate marker pose
def cvt_2Dmarker_measurements(ar_markers):

    marker2d_list = []

    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0,0,1], [0,-1,0], [1,0,0]])
        R_2_2p = np.matrix([[0,-1,0], [0,0,-1], [1,0,0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        #print('\n', R_2p_1p)
        yaw = -math.atan2(R_2p_1p[2,0], R_2p_1p[0,0])

        x, y = m.tvec[2][0] + 0.5, -m.tvec[0][0]
        print('x =', x, 'y =', y,'theta =', yaw)

        # remove any duplate markers
        dup_thresh = 2.0
        find_dup = False
        for m2d in marker2d_list:
            if grid_distance(m2d[0], m2d[1], x, y) < dup_thresh:
                find_dup = True
                break
        if not find_dup:
            marker2d_list.append((x,y,math.degrees(yaw)))

    return marker2d_list


#compute robot odometry based on past and current pose
def compute_odometry(curr_pose, cvt_inch=True):
    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees

    dx, dy = rotate_point(curr_x-last_x, curr_y-last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / 25.6, dy / 25.6

    return (dx, dy, diff_heading_deg(curr_h, last_h))

#particle filter functionality
class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)


async def run(robot: cozmo.robot.Robot):

    global flag_odom_init, last_pose
    global grid, gui

    # start streaming
    robot.camera.image_stream_enabled = True

    #start particle filter
    particlefilter = ParticleFilter(grid)

    ###################
    ############YOUR CODE HERE#################
    ###################

    # Not Test yet....
    my_go_to_pose(robot, goal[0], goal[1], goal[2], particlefilter)


def debug_print(message: str, debug = False):
    if debug:
        print(message)


def my_go_to_pose(robot, x, y, angle_z, particlefilter, debug = False):
    """Moves the robot to a pose relative to its current pose.
            Arguments:
            robot -- the Cozmo robot instance passed to the function
            x,y -- Desired position of the robot in millimeters
            angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    debug_print(f"[Go to Pose] Go to position ({x}, {y}), angle degree: {angle_z}", debug)
    if x == 0 and y == 0:
        robot.turn_in_place(angle_z).wait_for_completed()
        return

    world_old_position = robot.pose
    distance = math.sqrt(x**2 + y**2)
    while True:
        debug_print("======================================================", debug)
        world_new_position = robot.pose
        robot_pose = get_relative_pose(world_new_position, world_old_position)
        debug_print(f"[Go to Pose] Robot at ({robot_pose.position.x}, {robot_pose.position.y}), angle degree: {robot_pose.rotation.angle_z.degrees}", debug)

        delta_x = x - robot_pose.position.x
        delta_y = y - robot_pose.position.y

        rho = math.sqrt(delta_x**2 + delta_y**2)
        if delta_x == 0:
            alpha = get_number_signal(delta_y) * math.pi
        else:
            alpha = normalize_angle(math.atan2(delta_y, delta_x) - robot_pose.rotation.angle_z.radians)
        eta = normalize_angle(math.radians(angle_z) - robot_pose.rotation.angle_z.radians)

        debug_print("[Go to Pose] Errors:", debug)
        debug_print(f"[Go to Pose] rho: {rho}", debug)
        debug_print(f"[Go to Pose] alpha: {alpha}, degrees: {math.degrees(alpha)}", debug)
        debug_print(f"[Go to Pose] eta: {eta}, degrees: {math.degrees(eta)}", debug)

        r_markers_images = image_processing(robot)
        r_marker_list_raw = cvt_2Dmarker_measurements(r_markers_images)
        r_marker_list = []
        for m in r_marker_list_raw:
            r_marker_list.append(add_marker_measurement_noise(m,
                                                              trans_sigma=MARKER_TRANS_SIGMA,
                                                              rot_sigma=MARKER_ROT_SIGMA))

        odom = add_odometry_noise(compute_odometry(robot_pose),
                                  heading_sigma=ODOM_HEAD_SIGMA,
                                  trans_sigma=ODOM_TRANS_SIGMA)

        last_pose = robot_pose
        est_pose = particlefilter.update(odom, r_marker_list)
        if (abs(delta_x) < 1 and abs(delta_y) < 1):
            debug_print(f"[Go to Pose] Turn {math.degrees(eta)}", debug)
            robot.turn_in_place(math.degrees(eta)).wait_for_completed()
            debug_print("[Go to Pose] Stop", debug)
            robot.stop_all_motors()
            return

        p1 = 0.2
        # more focus on direction when far from goal
        # more focus on heading when near the goal
        if rho > distance / 5 * 3:
            p2 = 0.3
            p3 = 0.1
        elif rho < distance / 5:
            p2 = 0.1
            p3 = 0.3
        else:
            p2 = 0.2
            p3 = 0.2
        debug_print(f"[Go to Pose] p1: {p1}, p2: {p2}, p3: {p3}", debug)

        move_speed = p1 * rho
        rotation_speed = p2 * alpha + p3 * eta
        debug_print(f"[Go to Pose] Move Speed: {move_speed}, Rotation Degrees: {math.degrees(rotation_speed)}", debug)

        rotation_speed_mm = rotation_speed * get_distance_between_wheels() / 2
        left_speed = move_speed - rotation_speed_mm
        right_speed = move_speed + rotation_speed_mm
        debug_print(f"[Go to Pose] Left Speed: {left_speed}, Right Speed: {right_speed}", debug)

        robot.drive_wheels(left_speed, right_speed)
        if abs(left_speed) < 5 and abs(right_speed) < 5:
            #   When speed is not that farest, don't change the speed to often
            #   due Cozmo's motor and slip, might run into too long time and not stop
            time.sleep(1)
        else:
            time.sleep(0.1)


def get_relative_pose(object_pose, reference_frame_pose):
    # Homogeneous Transforms
    # | cos(ref_angle), -sin(ref_angle), ref_x | | related_x |   | obj_x |
    # | sin(ref_angle),  cos(ref_angle), ref_y | | related_y | = | obj_y |
    # |              0,               0,     1 | |         1 |   |     1 |
    #
    # ==>
    #
    # cos(ref_angle) * related_x - sin(ref_angle) * related_y + ref_x = obj_x
    # sin(ref_angle) * related_x + cos(ref_angle) * related_y + ref_y = obj_y
    #
    # ==>
    #
    # related_x = cos(ref_angle) * (obj_x - ref_x) + sin(ref_angle) * (obj_y - ref_y)
    # related_y = cos(ref_angle) * (obj_y - ref_y) - sin(ref_angle) * (obj_x - ref_x)
    obj_x = object_pose.position.x
    obj_y = object_pose.position.y
    obj_angle_z = object_pose.rotation.angle_z

    ref_x = reference_frame_pose.position.x
    ref_y = reference_frame_pose.position.y
    ref_angle_z = reference_frame_pose.rotation.angle_z

    newX = math.cos(ref_angle_z.radians) * (obj_x - ref_x) + \
        math.sin(ref_angle_z.radians) * (obj_y - ref_y)
    newY = math.cos(ref_angle_z.radians) * (obj_y - ref_y) - \
        math.sin(ref_angle_z.radians) * (obj_x - ref_x)
    newAngle = obj_angle_z - ref_angle_z

    return cozmo.util.pose_z_angle(newX, newY, 0, angle_z=newAngle, origin_id=object_pose._origin_id)


def get_distance_between_wheels():
    """Returns the distance between the wheels of the Cozmo robot in millimeters."""

    # The distance was determined by helper/get_distance_between_wheel.py.
    # Results is in ../lab7/helper/results/get_distance_between_wheel.txt
    #   The test test with different speed and it's mutiplicatoin
    #   Also for each speed, test with multiple time with the differnt of duration
    #   The total average is: 80.84659110426436
    return 80.85


def get_number_signal(number: float):
    if number == 0:
        return 1
    else:
        return number / abs(number)


def normalize_angle(radians):
    while radians < -math.pi:
        radians += 2 * math.pi
    while radians > math.pi:
        radians -= 2 * math.pi
    return radians


class CozmoThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid)
    gui.start()

