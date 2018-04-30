#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy
import math
import time
from cozmo.util import degrees


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


def get_relative_pose2(object_pose, reference_frame_pose):
    # Point R as Robot, Point C as Cube, x as world X axis, x' as robot X axis
    # tan(RCx) = (obj_y - ref_y) / (obj_x - ref_x)
    # theta = RCx = atan((obj_y - ref_y) / (obj_x - ref_x))
    # RCx' = ref_angle - RCx
    #
    # ==>
    #
    # related_x = cos(RCx') * sqrt((obj_y - ref_y)^2 + (obj_x - ref_x)^2)
    # related_y = -sin(RCx') * sqrt((obj_y - ref_y)^2 + (obj_x - ref_x)^2)
    #
    # This should return the save data as function get_relative_pose
    obj_x = object_pose.position.x
    obj_y = object_pose.position.y
    obj_angle_z = object_pose.rotation.angle_z

    ref_x = reference_frame_pose.position.x
    ref_y = reference_frame_pose.position.y
    ref_angle_z = reference_frame_pose.rotation.angle_z

    if (obj_x != ref_x):
        theta = math.atan((obj_y - ref_y) / (obj_x - ref_x))
    else:
        if (obj_y > ref_y):
            theta = math.pi / 2
        elif (obj_y < ref_y):
            theta = -math.pi / 2

    angle = ref_angle_z.radians - theta
    newX = math.cos(angle) * math.sqrt((obj_y - ref_y) *
                                       (obj_y - ref_y) + (obj_x - ref_x) * (obj_x - ref_x))
    newY = -math.sin(angle) * math.sqrt((obj_y - ref_y) *
                                        (obj_y - ref_y) + (obj_x - ref_x) * (obj_x - ref_x))
    newAngle = obj_angle_z - ref_angle_z

    return cozmo.util.pose_z_angle(newX, newY, 0, angle_z=newAngle, origin_id=object_pose._origin_id)


def find_relative_cube_pose(robot: cozmo.robot.Robot):
    '''Looks for a cube while sitting still, prints the pose of the detected cube
    in world coordinate frame and relative to the robot coordinate frame.'''

    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()
    cube = None

    while True:
        try:
            cube = robot.world.wait_for_observed_light_cube(timeout=30)
            if cube:
                print(f"Robot pose: {robot.pose}")
                print(f"Cube pose: {cube.pose}")
                print(
                    f"Cube pose in the robot coordinate frame: {get_relative_pose(cube.pose, robot.pose)}")
                print(
                    f"Cube pose in the robot coordinate frame: {get_relative_pose2(cube.pose, robot.pose)}")
        except asyncio.TimeoutError:
            print("Didn't find a cube")
        time.sleep(10)


if __name__ == '__main__':
    cozmo.run_program(find_relative_cube_pose)
