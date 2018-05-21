#!/usr/bin/env python3

'''Make Cozmo behave like a Braitenberg machine with virtual light sensors and wheels as actuators.

The following is the starter code for lab.
'''

import asyncio
import time
import cozmo
import cv2
import numpy as np
import sys

from collections import Counter
from imgclassification import ImageClassifier

from cozmo.util import degrees


async def run(robot: cozmo.robot.Robot):
    '''The core of the braitenberg machine program'''
    # Move lift down and tilt the head up

    await robot.set_head_angle(degrees(0)).wait_for_completed()
    await robot.set_lift_height(0.0).wait_for_completed()
    await robot.say_text('Game is on').wait_for_completed()
    print("Press CTRL-C to quit")

    camera = robot.camera
    print(camera.exposure_ms)
    print(camera.gain)
    # fixed_gain = (camera.config.min_gain + camera.config.max_gain) * 0.5
    # fixed_exposure_ms = 10
    camera.set_manual_exposure(20, camera.gain)

    await robot.say_text('Start Learning').wait_for_completed()
    img_clf = ImageClassifier()
    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)

    await robot.say_text("Let's do this").wait_for_completed()
    cumulation_predicted_labels = Counter()
    nothing_count = 0
    while True:
        # get camera image
        event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout = 30)

        # convert images into features
        test_data = img_clf.extract_image_features([np.asarray(event.image)])
        [ predicted_label ] = img_clf.predict_labels(test_data)

        if predicted_label != 'none':
            nothing_count = 0
            cumulation_predicted_labels[predicted_label] += 1
        else:
            nothing_count += 1
            if nothing_count > 50:
                nothing_count = 0
                cumulation_predicted_labels.clear()

        print(str(cumulation_predicted_labels))

        if sum(cumulation_predicted_labels.values()) >= 10:
            if cumulation_predicted_labels.most_common()[0][1] > 5:
                label = cumulation_predicted_labels.most_common()[0][0]
                await robot.say_text(label).wait_for_completed()
                await robot.play_anim_trigger(cozmo.anim.Triggers.AcknowledgeObject).wait_for_completed()
                nothing_count = 0
                cumulation_predicted_labels.clear()
            else:
                nothing_count = 0
                cumulation_predicted_labels.clear()

        time.sleep(.1)


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
