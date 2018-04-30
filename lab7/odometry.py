#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import sys

sys.path.insert(0, '../lab6')
from pose_transform import get_relative_pose

# Wrappers for existing Cozmo navigation functions

# Experiment warm up second for Cozmo Drive Wheels function,
# found by was determined by helper/understand_drive_wheels.py
# results is in helper/results/understand_drive_wheels.txt should be 0.9
# But for the length test, this should be 0.5
DRIVE_WHEELS_WARM_UP_SECOND = 0.9

def cozmo_drive_straight(robot, dist, speed):
    """Drives the robot straight.
            Arguments:
            robot -- the Cozmo robot instance passed to the function
            dist -- Desired distance of the movement in millimeters
            speed -- Desired speed of the movement in millimeters per second
    """
    robot.drive_straight(distance_mm(
        dist), speed_mmps(speed)).wait_for_completed()


def cozmo_turn_in_place(robot, angle, speed):
    """Rotates the robot in place.
            Arguments:
            robot -- the Cozmo robot instance passed to the function
            angle -- Desired distance of the movement in degrees
            speed -- Desired speed of the movement in degrees per second
    """
    robot.turn_in_place(degrees(angle), speed=degrees(
        speed)).wait_for_completed()


def cozmo_go_to_pose(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
            Arguments:
            robot -- the Cozmo robot instance passed to the function
            x,y -- Desired position of the robot in millimeters
            angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)),
                     relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs


def get_front_wheel_radius():
    """Returns the radius of the Cozmo robot's front wheel in millimeters."""
    # By watching the Cozmo to move straight and adjust the parameter,
    # found the length of circle is 86
    # The length was determined by helper/get_front_wheel_radium.py.
    # Results is in helper/results/get_front_wheel_radium.txt
    #   Radius is 13.687325105903
    return 13.69


def get_distance_between_wheels():
    """Returns the distance between the wheels of the Cozmo robot in millimeters."""

    # The distance was determined by helper/get_distance_between_wheel.py.
    # Results is in helper/results/get_distance_between_wheel.txt
    #   The test test with different speed and it's mutiplicatoin
    #   Also for each speed, test with multiple time with the differnt of duration
    #   The total average is: 80.84659110426436
    return 80.85


def rotate_front_wheel(robot, angle_deg):
    """Rotates the front wheel of the robot by a desired angle.
            Arguments:
            robot -- the Cozmo robot instance passed to the function
            angle_deg -- Desired rotation of the wheel in degrees
    """
    radians = math.radians(angle_deg)
    distance = radians * get_front_wheel_radius()
    cozmo_drive_straight(robot, distance, 50)


def my_drive_straight(robot, dist, speed, debug = False):
    """Drives the robot straight.
            Arguments:
            robot -- the Cozmo robot instance passed to the function
            dist -- Desired distance of the movement in millimeters
            speed -- Desired speed of the movement in millimeters per second
    """
    debug_print(f"[Drive Straight] Dist to {dist}, speed to {speed}", debug)
    if speed < 0:
        robot.say_text('Cannot do that').wait_for_completed()
        return
    if dist < 0 and speed > 0:
        speed = -speed
        dist = abs(dist)

    old_position = robot.pose
    new_position = old_position
    related_pose = get_relative_pose(new_position, old_position)
    rest = dist - abs(math.sqrt(related_pose.position.x**2 + related_pose.position.y**2))
    while rest > 0:
        debug_print(f"[Drive Straight] old_position {old_position}", debug)
        debug_print(f"[Drive Straight] new_position {new_position}", debug)
        debug_print(f"[Drive Straight] rest {rest}", debug)
        if abs(rest) < 5:
            break
        if rest < abs(speed):
            speed = get_number_signal(speed) * rest
            debug_print(f"[Drive Straight] lower speed to {speed}", debug)
        elif rest - abs(speed) < 10:     # Cannot move when distance is small
            speed = get_number_signal(speed) * (rest + 10)
            debug_print(f"[Drive Straight] higher speed to {speed}", debug)
        elif rest - abs(speed) < 30:     # Cannot move when distance is small
            speed = get_number_signal(speed) * rest
            debug_print(f"[Drive Straight] higher speed to {speed}", debug)
        robot.drive_wheels(speed, speed, 0, 0, duration=max(rest / abs(speed), 1))
        time.sleep(0.2)
        new_position = robot.pose
        related_pose = get_relative_pose(new_position, old_position)
        rest = dist - abs(math.sqrt(related_pose.position.x**2 + related_pose.position.y**2))
        if debug:
            debug_print(f"[Drive Straight] rest {rest}", debug)


def my_turn_in_place(robot, angle, speed, debug = False):
    """Rotates the robot in place.
            Arguments:
            robot -- the Cozmo robot instance passed to the function
            angle -- Desired distance of the movement in degrees
            speed -- Desired speed of the movement in degrees per second
    """
    debug_print(f"[Turn in Place] Angle to {angle}, speed to {speed}", debug)

    if speed <= 0:
        robot.say_text('Cannot do that').wait_for_completed()
    if abs(angle) <= 5:
        return

    while angle > 360:  # Reduce the turning angle
        angle -= 360
    while angle < -360: # Reduce the turning angle
        angle += 360
    if angle > 180:     # Reduce the turning angle
        angle = 360 - angle
        speed = -speed
    if angle < -180:    # Reduce the turning angle
        angle = 360 + angle
        speed = -speed
    if angle < 0 and speed > 0:
        speed = -speed
        angle = abs(angle)
    debug_print(f"[Turn in Place] Adjust angle to {angle}, speed to {speed}", debug)

    # If speed is positive turn left, otherwise, turn right
    old_angle = robot.pose.rotation.angle_z.degrees
    if old_angle < 0:
        old_angle += 360
    new_angle = old_angle
    while angle - (new_angle - old_angle) > 0:
        if speed > 0:
            delta = new_angle - old_angle
        else:
            delta = old_angle - new_angle
        if delta < 0:
            delta += 360
        rest = angle - delta
        debug_print(f'[Turn in Place] rest {rest}', debug)
        if abs(rest) < 7 or rest < 0:
            break
        if rest < abs(speed) - 1:
            speed = get_number_signal(speed) * rest
            debug_print(f'[Turn in Place] lower speed to {speed}', debug)
        if rest < 30 and rest - abs(speed) < 20: # Cannot turn when degree is small
            speed = get_number_signal(speed) * (rest + 15)
            debug_print(f'[Turn in Place] higher speed to {speed}', debug)
        if abs(speed) < 35: # Cannot turn when degree is small
            speed = get_number_signal(speed) * (35)
            debug_print(f'[Turn in Place] higher speed 2 to {speed}', debug)
        speed_mm = (get_distance_between_wheels() / 2) * math.radians(speed)
        robot.drive_wheels(-speed_mm, speed_mm, duration=DRIVE_WHEELS_WARM_UP_SECOND + max(rest / abs(speed), 1))
        time.sleep(0.2)
        new_angle = robot.pose.rotation.angle_z.degrees
        if new_angle < 0:
            new_angle += 360
        if debug:
            if speed > 0:
                delta = new_angle - old_angle
            else:
                delta = old_angle - new_angle
            if delta < 0:
                delta += 360
            rest = angle - delta
            debug_print(f'[Turn in Place] rest {rest}', debug)


def my_go_to_pose1(robot, x, y, angle_z, debug = False):
    """Moves the robot to a pose relative to its current pose.
            Arguments:
            robot -- the Cozmo robot instance passed to the function
            x,y -- Desired position of the robot in millimeters
            angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    distance = math.sqrt(x * x + y * y)
    angle = math.degrees(math.atan2(y, x))
    debug_print(distance, debug)
    debug_print(angle, debug)

    # Turn in place to point to new point
    my_turn_in_place(robot, angle, max(abs(angle / 2), 50), debug)
    # Move
    my_drive_straight(robot, distance, max(50, distance / 3), debug)
    # Turn in place to match angle_z
    angle = angle_z - angle
    my_turn_in_place(robot, angle, max(abs(angle / 2), 50), debug)


def my_go_to_pose2(robot, x, y, angle_z, debug = False):
    """Moves the robot to a pose relative to its current pose.
            Arguments:
            robot -- the Cozmo robot instance passed to the function
            x,y -- Desired position of the robot in millimeters
            angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    debug_print(f"[Go to Pose2] Go to position ({x}, {y}), angle degree: {angle_z}", debug)
    if x == 0 and y == 0:
        my_turn_in_place(robot, angle_z, angle_z, debug)
        return

    world_old_position = robot.pose
    distance = math.sqrt(x**2 + y**2)
    while True:
        debug_print("======================================================", debug)
        world_new_position = robot.pose
        robot_pose = get_relative_pose(world_new_position, world_old_position)
        debug_print(f"[Go to Pose2] Robot at ({robot_pose.position.x}, {robot_pose.position.y}), angle degree: {robot_pose.rotation.angle_z.degrees}", debug)

        delta_x = x - robot_pose.position.x
        delta_y = y - robot_pose.position.y

        rho = math.sqrt(delta_x**2 + delta_y**2)
        if delta_x == 0:
            alpha = get_number_signal(delta_y) * math.pi
        else:
            alpha = normalize_angle(math.atan2(delta_y, delta_x) - robot_pose.rotation.angle_z.radians)
        eta = normalize_angle(math.radians(angle_z) - robot_pose.rotation.angle_z.radians)

        debug_print("[Go to Pose2] Errors:", debug)
        debug_print(f"[Go to Pose2] rho: {rho}", debug)
        debug_print(f"[Go to Pose2] alpha: {alpha}, degrees: {math.degrees(alpha)}", debug)
        debug_print(f"[Go to Pose2] eta: {eta}, degrees: {math.degrees(eta)}", debug)

        if abs(rho) < 10 and abs(math.degrees(eta)) < 10:
            robot.stop_all_motors()
            debug_print("[Go to Pose2] Stop", debug)
            break
        elif abs(rho) < 10:
            #   Stop the movement and just turn to the right angle.
            #   If not stop at this time, due Cozmo's motor and slip, might run into too long time
            #   Also, if every parameter (p1,p2,p3) is good enough, this should not been used
            debug_print("[Go to Pose2] Stop", debug)
            robot.stop_all_motors()
            debug_print(f"[Go to Pose2] Turn {math.degrees(eta)}", debug)
            my_turn_in_place(robot, math.degrees(eta), abs(math.degrees(eta)), debug)
            break

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
        debug_print(f"[Go to Pose2] p1: {p1}, p2: {p2}, p3: {p3}", debug)

        move_speed = p1 * rho
        rotation_speed = p2 * alpha + p3 * eta
        debug_print(f"[Go to Pose2] Move Speed: {move_speed}, Rotation Degrees: {math.degrees(rotation_speed)}", debug)

        rotation_speed_mm = rotation_speed * get_distance_between_wheels() / 2
        left_speed = move_speed - rotation_speed_mm
        right_speed = move_speed + rotation_speed_mm
        debug_print(f"[Go to Pose2] Left Speed: {left_speed}, Right Speed: {right_speed}", debug)

        robot.drive_wheels(left_speed, right_speed)
        if abs(left_speed) < 5 and abs(right_speed) < 5:
            #   When speed is not that farest, don't change the speed to often
            #   due Cozmo's motor and slip, might run into too long time and not stop
            time.sleep(1)
        else:
            time.sleep(0.1)


def normalize_angle(radians):
    while radians < -math.pi:
        radians += 2 * math.pi
    while radians > math.pi:
        radians -= 2 * math.pi
    return radians


def my_go_to_pose3(robot, x, y, angle_z, debug = False):
    """Moves the robot to a pose relative to its current pose.
            Arguments:
            robot -- the Cozmo robot instance passed to the function
            x,y -- Desired position of the robot in millimeters
            angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    # ####
    # Find out when the angle is larger than 90 degree,
    # cozmo_go_to_pose() function will turn first
    # ####
    distance = math.sqrt(x * x + y * y)
    theta = get_number_signal(y) * math.degrees(math.atan2(abs(y), x))
    if abs(theta) > 90:
        turn_angle = theta
        debug_print(f"[Go to Pose3] Turn {turn_angle} first to have less movement", debug)
        my_turn_in_place(robot, turn_angle, max(abs(turn_angle / 2), 50), debug)
        #   Turn first and adjust the coordinate
        x = distance
        y = 0
        angle_z = angle_z - turn_angle
        debug_print(f"[Go to Pose3] After turn adjust x: {x}, y: {y}, angle: {angle_z}", debug)

    my_go_to_pose2(robot, x, y, angle_z, debug)


def get_number_signal(number: float):
    if number == 0:
        return 1
    else:
        return number / abs(number)


def debug_print(message: str, debug = False):
    if debug:
        print(message)


def run(robot: cozmo.robot.Robot):
    print(f"***** Front wheel radius     : {get_front_wheel_radius()}")
    print(f"***** Distance between wheels: {get_distance_between_wheels()}")

    # Example tests of the functions
    for angle in range(15, 181, 15):
        old_position = robot.pose
        rotate_front_wheel(robot, angle)
        new_position = robot.pose
        related_pose = get_relative_pose(new_position, old_position)
        moved = abs(math.sqrt(related_pose.position.x**2 + related_pose.position.y**2))
        distance = get_front_wheel_radius() * math.radians(angle)
        if abs(moved - distance) < 3:
            print(f'[rotate_front_wheel_test] Good in angle: {angle}')
        else:
            print(f'[rotate_front_wheel_test] Wrong in angle: {angle}, delta {abs(moved - distance)}')

    # cozmo_drive_straight(robot, 62, 50)
    for distance in range(50, 100, 15):
        for speed in range(20, 50, 10):
            for signal in range(-1, 2, 2):
                dist = signal * distance
                old_position = robot.pose
                my_drive_straight(robot, dist, speed)
                new_position = robot.pose
                related_pose = get_relative_pose(new_position, old_position)
                moved = abs(math.sqrt(related_pose.position.x**2 + related_pose.position.y**2))
                if abs(moved - distance) < 10:
                    print(f'[my_drive_straight_test] Good in distance: {dist}, speed: {speed}')
                else:
                    print(f'[my_drive_straight_test] Wrong in distance: {dist}, speed: {speed}, delta {moved - distance}')

    # cozmo_turn_in_place(robot, 60, 30)
    for orignal_angle in range(30, 31, 30):
        for speed in range(30, 61, 15):
            for signal in range(-1, 2, 2):
                angle = signal * orignal_angle
                old_angle = robot.pose.rotation.angle_z.degrees
                if old_angle < 0:
                    old_angle += 360
                my_turn_in_place(robot, angle, speed)
                new_angle = robot.pose.rotation.angle_z.degrees
                if new_angle < 0:
                    new_angle += 360
                if (new_angle > old_angle):
                    delta = new_angle - old_angle
                else:
                    delta = old_angle - new_angle
                if delta > 180:
                    delta -= 360
                if abs(abs(delta) - abs(angle)) < 10:
                    print(f'[my_turn_in_place_test] Good in angle: {angle}, speed: {speed}')
                else:
                    print(f'[my_turn_in_place_test] Wrong in angle: {angle}, speed: {speed}, delta {abs(delta) - abs(angle)}')

    for x in range(-100, 101, 100):
        for y in range(-100, 101, 100):
            for angle in range(-45, 90, 45):
                old_pose = robot.pose
                my_go_to_pose1(robot, x, y, angle)
                new_pose = robot.pose
                related_pose = get_relative_pose(new_pose, old_pose)
                print(f'[Go to Pose 1 Test] Moved x {related_pose.position.x}), y: {related_pose.position.y}, angle: {related_pose.rotation.angle_z.degrees})')
                delta_x = related_pose.position.x - x
                delta_y = related_pose.position.y - y
                delta_angle = related_pose.rotation.angle_z.degrees - angle
                if abs(delta_x) < 15 and abs(delta_y) < 15 and abs(delta_angle) < 5:
                    print(f'[Go to Pose 1 Test] Good in x: {x}, y: {y}, angle: {angle}')
                else:
                    print(f'[Go to Pose 1 Test] Wrong in x: {x} (delta {delta_x}), y: {y} (delta {delta_y}), angle: {angle} (delta {delta_angle})')

    for x in range(-100, 101, 100):
        for y in range(-100, 101, 100):
            for angle in range(-45, 90, 45):
                old_pose = robot.pose
                my_go_to_pose2(robot, x, y, angle)
                new_pose = robot.pose
                related_pose = get_relative_pose(new_pose, old_pose)
                print(f'[Go to Pose 2 Test] Moved x {related_pose.position.x}), y: {related_pose.position.y}, angle: {related_pose.rotation.angle_z.degrees})')
                delta_x = related_pose.position.x - x
                delta_y = related_pose.position.y - y
                delta_angle = related_pose.rotation.angle_z.degrees - angle
                if abs(delta_x) < 10 and abs(delta_y) < 10 and abs(delta_angle) < 10:
                    print(f'[Go to Pose 2 Test] Good in x: {x}, y: {y}, angle: {angle}')
                else:
                    print(f'[Go to Pose 2 Test] Wrong in x: {x} (delta {delta_x}), y: {y} (delta {delta_y}), angle: {angle} (delta {delta_angle})')

    # cozmo_go_to_pose(robot, 100, 0, 45)
    # cozmo_go_to_pose(robot, 100, 100, 45)
    # cozmo_go_to_pose(robot, 100, -100, 45)
    # cozmo_go_to_pose(robot, -100, -100, 45)
    # cozmo_go_to_pose(robot, 0, -150, 45)

    for x in range(-100, 101, 100):
        for y in range(-100, 101, 100):
            for angle in range(-45, 90, 45):
                old_pose = robot.pose
                my_go_to_pose3(robot, x, y, angle)
                new_pose = robot.pose
                related_pose = get_relative_pose(new_pose, old_pose)
                print(f'[Go to Pose 3 Test] Moved x {related_pose.position.x}), y: {related_pose.position.y}, angle: {related_pose.rotation.angle_z.degrees})')
                delta_x = related_pose.position.x - x
                delta_y = related_pose.position.y - y
                delta_angle = related_pose.rotation.angle_z.degrees - angle
                if abs(delta_x) < 15 and abs(delta_y) < 15 and abs(delta_angle) < 10:
                    print(f'[Go to Pose 3 Test] Good in x: {x}, y: {y}, angle: {angle}')
                else:
                    print(f'[Go to Pose 3 Test] Wrong in x: {x} (delta {delta_x}), y: {y} (delta {delta_y}), angle: {angle} (delta {delta_angle})')


if __name__ == '__main__':
    cozmo.run_program(run)
