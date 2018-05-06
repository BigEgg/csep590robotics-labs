from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo

from cozmo.objects import LightCube1Id, LightCube2Id, LightCube3Id
from cozmo.util import degrees, Angle, Pose

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
    start_pose = grid.getStart()
    goal_pose = grid.getGoals()[0]

    frontier = PriorityQueue()
    frontier.put((0, start_pose))
    came_from = {}
    cost_so_far = {}
    came_from[start_pose] = None
    cost_so_far[start_pose] = 0

    while not frontier.empty():
        current = frontier.get()[1]
        grid.addVisited(current)

        if current == goal_pose:
            path = reconstruct_path(came_from, start_pose, goal_pose)
            grid.setPath(path)
            return

        for next in grid.getNeighbors(current):
            next_pose, next_weight = next[0], next[1]
            new_cost = cost_so_far[current] + next_weight

            if next_pose not in cost_so_far or new_cost < cost_so_far[next_pose]:
                cost_so_far[next_pose] = new_cost
                priority = new_cost + heuristic(next_pose, goal_pose)
                frontier.put((priority, next_pose))
                came_from[next_pose] = current

    raise ValueError('No Path Found')


def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]

    path.append(start)
    path.reverse()
    return path


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    (current_x, current_y) = current
    (goal_x, goal_y) = goal
    return math.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)


def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """

    global grid, stopevent

    grid.addGoal((grid.height / 2, grid.width / 2))
    found_goal = False
    astar(grid, heuristic)
    path_index = 0
    path = grid.getPath()
    grid_init_start_pose = grid.getStart()
    robot.say_text('Game is on').wait_for_completed()
    while not stopevent.is_set():
        new_cube = search_cube(robot, grid)
        if not new_cube == None:
            add_obstacle(grid, new_cube, grid_init_start_pose)    # Add the obstacle for all cubes that had been found
            if new_cube.cube_id == LightCube1Id:
                new_cube.set_lights(cozmo.lights.blue_light)
                robot.say_text('Found the Goal').wait_for_completed()
                get_goal(grid, new_cube, grid_init_start_pose)   # Update the goal coordinate while found cube 1
                found_goal = True
            else:
                new_cube.set_lights(cozmo.lights.red_light)
                robot.say_text('Found an Obstacle').wait_for_completed()

            # Replanning the path for
            robot.say_text('Replanning').wait_for_completed()
            grid.clearVisited()
            grid.setStart(position_to_grid(grid, robot.pose.position.x, robot.pose.position.y, grid_init_start_pose))
            astar(grid, heuristic)
            path_index = 0
            path = grid.getPath()

        if path_index >= len(path): # At the goal position
            if not found_goal:      # At the center of grid
                robot.turn_in_place(Angle(degrees=30))
            else:                   # Arrived the final place
                robot.say_text('Arrived').wait_for_completed()
                break

        current_pose = grid.getPath()[path_index]
        next_pose = grid.getPath()[path_index + 1]
        x = (next_pose[0] - current_pose[0]) * grid.scale
        y = (next_pose[1] - current_pose[1]) * grid.scale
        degree = (0 if x == 0 else degrees(math.atan(y / x))) - robot.pose.rotation.angle_z.degrees
        robot.turn_in_place(Angle(degrees=degree))
        robot.go_to_pose(Pose(math.sqrt(x**2, y**2), 0, 0, angle_z=0), relative_to_robot=True).wait_for_completed()

    stopevent.wait()


def search_cube(robot: cozmo.robot.Robot, grid: CozGrid):
    robot.say_text('searching').wait_for_completed()
    cube = robot.world.wait_for_observed_light_cube(timeout=5, include_existing=False)
    if cube:
        robot.say_text('Found a Cube').wait_for_completed()
        return cube
    else:
        return None


def add_obstacle(grid: CozGrid, cube: cozmo.objects.LightCube, grid_init_start_pose):
    cube_size = 50
    for x in range(-cube_size, cube_size + 1, grid.scale):
        for y in range(-cube_size, cube_size + 1, grid.scale):
            (obstacle_x, obstacle_y) = rotate_point(x, y, cube.pose.rotation.angle_z.degrees)
            grid.addObstacle(
                position_to_grid(
                    grid,
                    cube.pose.position.x + obstacle_x,
                    cube.pose.position.y + obstacle_y,
                    grid_init_start_pose)
            )


def position_to_grid(grid: CozGrid, x, y, grid_init_start_pose):
    (init_x, init_y) = grid_init_start_pose
    return (x / grid.scale + init_x,
            y / grid.scale + init_y)


def get_goal(grid: CozGrid, cube: cozmo.objects.LightCube, grid_init_start_pose):
    cube_size = 50
    # Cube right and back will be the picture, choose right this time
    (goal_x, goal_y) = rotate_point(0, -cube_size, cube.pose.rotation.angle_z.degrees)
    grid.clearGoals()
    grid.addGoal(
        position_to_grid(
            grid,
            cube.pose.position.x + goal_x,
            cube.pose.position.y + goal_y,
            grid_init_start_pose)
    )


def rotate_point(x, y, heading_deg):
    c = math.cos(math.radians(heading_deg))
    s = math.sin(math.radians(heading_deg))
    xr = x * c + y * -s
    yr = x * s + y * c
    return xr, yr

######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()
