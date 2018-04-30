import cozmo
import math
import time
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps


def cozmo_drive_straight(robot, dist, speed):
    """Drives the robot straight.
            Arguments:
            robot -- the Cozmo robot instance passed to the function
            dist -- Desired distance of the movement in millimeters
            speed -- Desired speed of the movement in millimeters per second
    """
    robot.drive_straight(distance_mm(
        dist), speed_mmps(speed)).wait_for_completed()


def get_front_wheel_radius(robot: cozmo.robot.Robot):
    """Returns the radius of the Cozmo robot's front wheel in millimeters."""

    length = 86
    old_position = robot.pose.position.x
    cozmo_drive_straight(robot, length, 30)
    # cozmo_drive_straight(robot, length * 2, 30)
    # cozmo_drive_straight(robot, length * 3, 30)
    # cozmo_drive_straight(robot, length * 4, 30)
    # cozmo_drive_straight(robot, length * 5, 30)
    new_position = robot.pose.position.x
    print(f'Radius is {length / (math.pi * 2)}, moved: {new_position - old_position}')


if __name__ == '__main__':
    cozmo.run_program(get_front_wheel_radius)
