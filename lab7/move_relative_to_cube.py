#!/usr/bin/env python3

'''
This is starter code for Lab 7.

'''

import asyncio
import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import sys

from odometry import cozmo_go_to_pose
from odometry import my_go_to_pose3
from odometry import my_turn_in_place
sys.path.insert(0, '../lab6')
from pose_transform import get_relative_pose


def move_relative_to_cube(robot: cozmo.robot.Robot):
    '''Looks for a cube while sitting still, when a cube is detected it
    moves the robot to a given pose relative to the detected cube pose.'''

    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()
    cube = None

    while True:
        try:
            robot.say_text('searching').wait_for_completed()
            cube = robot.world.wait_for_observed_light_cube(timeout=5)
            if cube:
                robot.say_text('Found it').wait_for_completed()
                break
        except asyncio.TimeoutError:
            my_turn_in_place(robot, 30, 50, True)

    relative_pose = get_relative_pose(cube.pose, robot.pose)
    print(f"Found a cube, pose in the robot coordinate frame: {relative_pose}")

    my_go_to_pose3(robot, relative_pose.position.x, relative_pose.position.y, relative_pose.rotation.angle_z.degrees)

    robot.say_text('Arrived').wait_for_completed()
    robot.play_anim_trigger(cozmo.anim.Triggers.FistBumpSuccess).wait_for_completed()


if __name__ == '__main__':
    cozmo.run_program(move_relative_to_cube, use_viewer = True, force_viewer_on_top = True)
