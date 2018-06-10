from cleanTableRobot import CleanTableRobot
from humanInLoopReinforcementLearningAgent import HumanInLoopReinforcementLearningAgent

import time

import cozmo

async def cleanTableRobotBehavior(cozmoRobot: cozmo.robot.Robot):
    cleanTableRobot = CleanTableRobot(cozmoRobot)
    actionFn = lambda state: cleanTableRobot.getActions(state)
    qLearnOpts = { 'actionFn': actionFn }

    agent = HumanInLoopReinforcementLearningAgent(**qLearnOpts)

    await cozmoRobot.say_text('Start Learning').wait_for_completed()
    await cleanTableRobot.initialize()
    while True:      # Learning Mode
        currentState = cleanTableRobot.getStatus()
        action = agent.getAction(currentState)
        if action == 'exit':
            break
        await cleanTableRobot.takeAction(action)
        newState = cleanTableRobot.getStatus()

        reward = await cleanTableRobot.waitForResponse()
        agent.update(currentState, action, newState, reward)
    await cozmoRobot.say_text('Learning Success').wait_for_completed()
    time.sleep(10)

    await cozmoRobot.say_text("Let's Learn again").wait_for_completed()
    await cleanTableRobot.initialize()
    while True:      # Learning Mode
        currentState = cleanTableRobot.getStatus()
        action = agent.getAction(currentState)
        if action == 'exit':
            break
        await cleanTableRobot.takeAction(action)
        newState = cleanTableRobot.getStatus()

        reward = await cleanTableRobot.waitForResponse()
        agent.update(currentState, action, newState, reward)
    await cozmoRobot.say_text('Learning Success').wait_for_completed()
    time.sleep(10)

    await cozmoRobot.say_text("Let's do it.").wait_for_completed()
    await cleanTableRobot.initialize()
    while True:      # Actoin Mode
        currentState = cleanTableRobot.getStatus()
        action = agent.getAction(currentState)
        if action == 'exit':
            break
        await cleanTableRobot.takeAction(action)
        newState = cleanTableRobot.getStatus()
    await cozmoRobot.say_text("Complete.").wait_for_completed()

# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    cozmo.run_program(cleanTableRobotBehavior, use_viewer = True, force_viewer_on_top = True)
