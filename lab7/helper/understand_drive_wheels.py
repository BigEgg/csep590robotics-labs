#!/usr/bin/env python3

import asyncio
import sys
import cv2
import time
import cozmo
from cozmo.util import degrees, speed_mmps, distance_mm, Angle

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


# Define a decorator as a subclass of Annotator; displays battery voltage
class PositionAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        x = self.world.robot.pose.position.x
        text = cozmo.annotate.ImageText(f"X: {x}", color="green")
        text.render(d, bounds)


async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    # add annotators for position
    robot.world.image_annotator.add_annotator('position', PositionAnnotator)
    warm_up = await get_drive_wheels_warm_up(robot)
    print(f'Warm up time by duration is {warm_up}')
    warm_up = await get_drive_wheels_warm_up_by_length(robot)
    print(f'Warm up time by distance is {warm_up}')


async def get_drive_wheels_warm_up(robot: cozmo.robot.Robot):
    for duration in range(1, 15, 1):
        old_position = robot.pose.position.x
        await robot.drive_wheels(20, 20, duration=duration / 10)
        new_position = robot.pose.position.x
        if new_position > old_position:
            print(f'Start moving when duration is {duration / 10}, distance: {new_position - old_position}')
            return (duration - 1) / 10
        else:
            print(f'No move when duration is {duration / 10}')
        time.sleep(1)


async def get_drive_wheels_warm_up_by_length(robot: cozmo.robot.Robot):
    distance = 50
    speed = 25
    for duration in range(1, 15, 1):
        old_position = robot.pose.position.x
        await robot.drive_wheels(speed, speed, duration=(duration / 10) + (distance / speed))
        new_position = robot.pose.position.x
        if (new_position - old_position > distance - 3) and (new_position - old_position < distance + 3):
            print(f'Moved enought distance, duration: {duration / 10}')
            return duration / 10
        else:
            print(f'Not moved enought distance when duration is {duration / 10}, delta: {abs(new_position - old_position - distance)}')
        time.sleep(1)


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer=True, force_viewer_on_top=True)
