#!/usr/bin/env python3

import asyncio
import sys

import cv2

import cozmo
from cozmo.util import degrees, speed_mmps, distance_mm, Angle

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


# Define a decorator as a subclass of Annotator; displays battery voltage
class AngleAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        angle = self.world.robot.pose.rotation.angle_z
        text = cozmo.annotate.ImageText(f"Angle {angle}", color="green")
        text.render(d, bounds)


async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    # add annotators for angle
    robot.world.image_annotator.add_annotator('angle', AngleAnnotator)

    print(robot.pose.rotation.angle_z)
    await robot.turn_in_place(Angle(degrees=1080), speed=Angle(degrees=20)).wait_for_completed()
    print(robot.pose.rotation.angle_z)

    await robot.drive_wheels(20, 0, duration=20)


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer=True, force_viewer_on_top=True)
