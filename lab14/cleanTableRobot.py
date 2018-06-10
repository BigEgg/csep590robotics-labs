
import asyncio
import sys
import math

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps, Position, Pose, Angle
from cozmo.objects import LightCube, LightCube1Id, LightCube2Id, LightCube3Id

class CleanTableRobot():
    def __init__(self, robot: cozmo.robot.Robot):
        self.robot = robot
        self.isPickupTrash = False
        self.placedTrash = False
        self.cube1 = robot.world.get_light_cube(LightCube1Id)  # looks like a paperclip
        self.cube2 = robot.world.get_light_cube(LightCube2Id)  # looks like a lamp / heart
        self.cube3 = robot.world.get_light_cube(LightCube3Id)  # looks like the letters 'ab' over 'T'
        self.trash = self.cube1
        self.dustBin = self.cube2
        self.correctTabCube = self.cube3
        self.wrongTabCube = self.cube2

    async def initialize(self):
        await self.robot.set_head_angle(degrees(0)).wait_for_completed()
        await self.robot.set_lift_height(0.0).wait_for_completed()

        lookaround = self.robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
        cubes = await self.robot.world.wait_until_observe_num_objects(num=3, object_type=cozmo.objects.LightCube, timeout=60)
        lookaround.stop()

        if len(cubes) < 3:
            await self.robot.say_text('Cannot find 3 cubes').wait_for_completed()
            print(f'Error: need 3 Cubes but only found {len(cubes)} Cube(s)')
            await self.robot.play_anim_trigger(cozmo.anim.Triggers.MajorFail).wait_for_completed()
            sys.exit(1)

        await self.robot.play_anim_trigger(cozmo.anim.Triggers.BlockReact).wait_for_completed()
        self.trash.set_lights(cozmo.lights.blue_light)
        self.wrongTabCube.set_lights(cozmo.lights.red_light)
        self.correctTabCube.set_lights(cozmo.lights.green_light)

    def getActions(self, status):
        (distanceToTrash, distanceToDustBin, isPickupTrash) = status
        if self.placedTrash:    # stage exit
            print("Stage exit")
            return [ 'exit' ]
        if not isPickupTrash and distanceToTrash > 2:   # stage 1
            print("Stage 1")
            return [ 'moveCloserToTrash', 'moveFurtherToTrash', 'moveCloserToDustBin', 'moveFurtherToDustBin' ]
        if not isPickupTrash and distanceToTrash <= 2:  # stage 2
            print("Stage 2")
            return [ 'pickUpTrash', 'moveFurtherToTrash', 'moveCloserToDustBin', 'moveFurtherToDustBin' ]
        if isPickupTrash and distanceToDustBin > 2:     # stage 3
            print("Stage 3")
            return [ 'moveCloserToDustBin', 'moveFurtherToDustBin' ]
        if isPickupTrash and distanceToDustBin <= 2:    # stage 4
            print("Stage 4")
            return [ 'putTrashIntoDustBin', 'moveFurtherToDustBin' ]

        return []

    def getStatus(self):
        distanceToTrash = self.__getDistance__(self.trash.pose.position, self.robot.pose.position)
        distanceToDustBin = self.__getDistance__(self.dustBin.pose.position, self.robot.pose.position)
        return (int(distanceToTrash / 50), int(distanceToDustBin / 50), self.isPickupTrash)

    async def takeAction(self, actionType: str):
        if actionType == 'moveCloserToTrash':
            await self.__moveCloserToTrash__()
        elif actionType == 'moveFurtherToTrash':
            await self.__moveFurtherToTrash__()
        elif actionType == 'moveCloserToDustBin':
            await self.__moveCloserToDustBin__()
        elif actionType == 'moveFurtherToDustBin':
            await self.__moveFurtherToDustBin__()
        elif actionType == 'pickUpTrash':
            await self.__pickUpTrash__()
        elif actionType == 'putTrashIntoDustBin':
            await self.__putTrashIntoDustBin__()

    async def waitForResponse(self):
        await self.robot.say_text('Is that correct step?').wait_for_completed()

        try:
            self.wrongTabCube.set_lights_off()
            print('Waiting for cube to be tapped')
            await self.correctTabCube.wait_for_tap(timeout=5)
            await self.robot.say_text('Haha').wait_for_completed()
            print('Correct Tab Cube tapped')
            return 1
        except asyncio.TimeoutError:
            print('correct Tab Cube not tapped')
        finally:
            self.wrongTabCube.set_lights(cozmo.lights.red_light)

        await self.robot.say_text('So is that wrong step?').wait_for_completed()
        try:
            self.correctTabCube.set_lights_off()
            print('Waiting for cube to be tapped')
            await self.wrongTabCube.wait_for_tap(timeout=5)
            await self.robot.say_text('Oh').wait_for_completed()
            print('Wrong Tab Cube tapped')
            return -1
        except asyncio.TimeoutError:
            print('Wrong Tab Cube not tapped')
        finally:
            self.correctTabCube.set_lights(cozmo.lights.green_light)

        print('No-one tapped our cube :-(')
        return 0


    def __getDistance__(self, position1: Position, position2: Position):
        return math.sqrt((position1.x - position2.x) ** 2 + (position1.y - position2.y) ** 2)

    async def __moveCloserToTrash__(self):
        print("Action: ===moveCloserToTrash===")
        distanceToTrash = self.__getDistance__(self.trash.pose.position, self.robot.pose.position)
        print(f"    distance to trash {distanceToTrash}")
        return await self.robot.go_to_object(self.trash, distance_mm(distanceToTrash - 50)).wait_for_completed()

    async def __moveFurtherToTrash__(self):
        print("Action: ===moveFurtherToTrash===")
        distanceToTrash = self.__getDistance__(self.trash.pose.position, self.robot.pose.position)
        print(f"    distance to trash {distanceToTrash}")

        await self.robot.go_to_object(self.trash, distance_mm(distanceToTrash - 50)).wait_for_completed()
        return await self.robot.drive_straight(distance_mm(-50), speed_mmps(50)).wait_for_completed()

    async def __moveCloserToDustBin__(self):
        print("Action: ===moveCloserToDustBin===")
        distanceToDustBin = self.__getDistance__(self.dustBin.pose.position, self.robot.pose.position)
        print(f"    distance to dust bin {distanceToDustBin}")
        return await self.robot.go_to_object(self.dustBin, distance_mm(distanceToDustBin - 50)).wait_for_completed()

    async def __moveFurtherToDustBin__(self):
        print("Action: ===moveFurtherToDustBin===")
        distanceToDustBin = self.__getDistance__(self.dustBin.pose.position, self.robot.pose.position)
        print(f"    distance to dust bin {distanceToDustBin}")

        await self.robot.go_to_object(self.dustBin, distance_mm(distanceToDustBin - 50)).wait_for_completed()
        return await self.robot.drive_straight(distance_mm(-50), speed_mmps(50)).wait_for_completed()

    async def __pickUpTrash__(self):
        print("Action: ===pickUpTrash===")
        current_action = self.robot.pickup_object(self.trash, num_retries=3)
        await current_action.wait_for_completed()
        if current_action.has_failed:
            code, reason = current_action.failure_reason
            result = current_action.result
            print("Pickup Cube failed: code=%s reason='%s' result=%s" % (code, reason, result))
            return
        self.isPickupTrash = True

    async def __putTrashIntoDustBin__(self):
        print("Action: ===putTrashIntoDustBin===")
        current_action = self.robot.place_on_object(self.dustBin, num_retries=3)
        await current_action.wait_for_completed()
        if current_action.has_failed:
            code, reason = current_action.failure_reason
            result = current_action.result
            print("Place On Cube failed: code=%s reason='%s' result=%s" % (code, reason, result))
            return
        self.isPickupTrash = False
        self.placedTrash = True

    def get_relative_pose(self, object_pose, reference_frame_pose):
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
