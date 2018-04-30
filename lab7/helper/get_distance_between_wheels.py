#!/usr/bin/env python3

import asyncio
import cozmo
import numpy
import math
import time
from cozmo.util import degrees, distance_mm, speed_mmps, Angle

async def test(robot: cozmo.robot.Robot):
    #   Test with speed 25, 30, 35
    averages = []
    for test_speed in range(25, 40, 5):
        averages.append(await test_with_speed(robot, test_speed))
    print(f'Average B: {mean(averages)}')


async def test_with_speed(robot: cozmo.robot.Robot, test_speed: int):
    #   For each speed test with a muliplication to have a better data
    averages = []
    for speed in range(test_speed, test_speed * 4, test_speed):
        averages.append(await get_distance_between_wheels(robot, speed))
    print(f'Average B: {mean(averages)} with test speed from {test_speed}, {test_speed * 2}, {test_speed * 3}')
    return mean(averages)


async def get_distance_between_wheels(robot: cozmo.robot.Robot, speed: int, debug: bool = False):
    robot.move_lift(-3)
    await robot.set_head_angle(degrees(0)).wait_for_completed()

    bArray = []
    last_phi = Angle(degrees=0)
    for duration in range(1, 7):
        # Turn right with center of wheel as point, b is the distance between wheels
        # So
        #   phi * (b / 2) = speed * time
        # And because
        #   phi = old_robot_angle - new_robot_angle
        #   ==>
        #   b = speed * time / (old_robot_angle - new_robot_angle) * 2
        oldPose = robot.pose
        await robot.drive_wheels(speed, -speed, duration=duration)
        newPose = robot.pose
        phi = normalize_angle_and_minus(newPose.rotation.angle_z, oldPose.rotation.angle_z)

        #   Use the delta of the value to ingore the warm up time or friction
        delta_phi = phi - last_phi
        last_phi = phi
        if duration < 3:    #   Skip the test data because when duration less than 3, the value is not accurate
            continue

        if debug:
            print(f"phi: {phi}")
            print(f"delta_phi: {delta_phi}")

        if delta_phi.abs_value.radians < 0.1:
            print('Wrong data, ignore')
            continue

        b = (speed * 1) / delta_phi.abs_value.radians * 2
        if b > 100 or b < 40:
            print('Wrong data, ignore')
            continue

        print(f"With speed {speed} and time {duration}, the distance between wheels: by phi {b}")
        bArray.append(b)
    print(f'Average B: {mean(bArray)} with speed {speed}')
    return mean(bArray)


def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)


def normalize_angle_and_minus(angle1: Angle, angle2: Angle):
    degrees1 = angle1.degrees
    degrees2 = angle2.degrees

    if degrees1 < 0:
        degrees1 += 360
    if degrees2 < 0:
        degrees2 += 360

    if degrees1 < degrees2:
        return Angle(degrees=degrees2 - degrees1)
    else:
        return Angle(degrees=degrees2 + 360 - degrees1)


if __name__ == '__main__':
    cozmo.run_program(test)
