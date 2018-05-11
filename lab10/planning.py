from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
import asyncio

from cozmo.objects import LightCube1Id, LightCube2Id, LightCube3Id
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
    start_pose = grid.getStart()
    goal_poses = grid.getGoals()
    shortest_path = []
    shortest_path_visited = set()
    for goal_pose in goal_poses:
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
                if len(path) < len(shortest_path) or len(shortest_path) == 0:
                    shortest_path = path
                    shortest_path_visited = set(grid.getVisited())
                continue

            for next in grid.getNeighbors(current):
                next_pose, next_weight = next[0], next[1]
                new_cost = cost_so_far[current] + next_weight

                if next_pose not in cost_so_far or new_cost < cost_so_far[next_pose]:
                    cost_so_far[next_pose] = new_cost
                    priority = new_cost + heuristic(next_pose, goal_pose)
                    frontier.put((priority, next_pose))
                    came_from[next_pose] = current
        grid.clearPath()
        grid.clearVisited()

    if len(shortest_path) > 0:
        grid.setPath(shortest_path)
        for visitied_point in shortest_path_visited:
            grid.addVisited(visitied_point)
    else:
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
    global found_cubes
    found_cubes = []
    goal_degrees = []

    grid.addGoal((grid.width / 2, grid.height / 2))
    found_goal = False
    astar(grid, heuristic)
    path = grid.getPath()
    path_index = 0
    grid_init_start_pose = grid.getStart()
    (robot_grid_x, robot_grid_y) = grid.getStart()

    robot.set_head_angle(degrees(0)).wait_for_completed()
    robot.set_lift_height(1).wait_for_completed()
    robot.say_text('Game is on').wait_for_completed()

    while not stopevent.is_set():
        new_cube = search_cube(robot, grid)
        if not new_cube == None:
            grid.clearStart()
            grid.clearVisited()
            grid.clearPath()
            grid.setStart((robot_grid_x, robot_grid_y))
            add_obstacle(grid, new_cube, grid_init_start_pose)    # Add the obstacle for all cubes that had been found
            if new_cube.cube_id == LightCube1Id:
                new_cube.set_lights(cozmo.lights.blue_light)
                goal_degrees = set_goal(grid, new_cube, grid_init_start_pose)   # Update the goal coordinate while found cube 1
                robot.say_text("It's the Goal").wait_for_completed()
                found_goal = True
            else:
                new_cube.set_lights(cozmo.lights.red_light)
                robot.say_text("It's an Obstacle").wait_for_completed()

            # Replanning the path for
            robot.say_text('Replanning').wait_for_completed()
            try:
                astar(grid, heuristic)
            except:
                robot.say_text("Cannot go to that place").wait_for_completed()
                return
            path_index = 0
            path = grid.getPath()

        path_index += 1
        if path_index == len(path): # At the goal position
            if not found_goal:      # At the center of grid
                path_index -= 1
                robot.turn_in_place(Angle(degrees=-30)).wait_for_completed()
                continue
            else:                   # Arrived the final place
                goal_degree = goal_degrees[f"{robot_grid_x}, {robot_grid_y}"]
                robot.turn_in_place(Angle(degrees=normalize_angle(goal_degree - robot.pose.rotation.angle_z.degrees))).wait_for_completed()
                robot.say_text('Arrived').wait_for_completed()
                break

        current_pose = path[path_index - 1]
        next_pose = path[path_index]
        robot_grid_x += next_pose[0] - current_pose[0]
        robot_grid_y += next_pose[1] - current_pose[1]

        x = (next_pose[0] - current_pose[0]) * grid.scale
        y = (next_pose[1] - current_pose[1]) * grid.scale
        degree = ((90 * y / abs(y)) if x == 0 else math.degrees(math.atan2(y, x))) - robot.pose.rotation.angle_z.degrees
        robot.turn_in_place(Angle(degrees=normalize_angle(degree))).wait_for_completed()
        robot.drive_straight(distance_mm(math.sqrt(x**2 + y**2)), speed_mmps(50), should_play_anim=False).wait_for_completed()

    stopevent.wait()


def normalize_angle(degrees):
    while degrees < -180:
        degrees += 360
    while degrees > 180:
        degrees -= 360
    return degrees


def search_cube(robot: cozmo.robot.Robot, grid: CozGrid):
    global found_cubes
    if len(found_cubes) == 3: # Already found all cubes
        return None
    try:
        robot.say_text('searching').wait_for_completed()
        cube = robot.world.wait_for_observed_light_cube(timeout=2, include_existing=False)
        if cube and cube.cube_id not in found_cubes:
            found_cubes.append(cube.cube_id)        # Robot will always return found cubes, need filter by hand
            robot.say_text('Found a Cube').wait_for_completed()
            return cube
    except asyncio.TimeoutError:
        return None

# Cube is same size as 2 grid scale but also need have another scale for robot, so 3 grid scale
CUBE_SIZE = 2

def add_obstacle(grid: CozGrid, cube: cozmo.objects.LightCube, grid_init_start_pose):
    for x in range(-CUBE_SIZE, CUBE_SIZE + 1, 1):
        for y in range(-CUBE_SIZE, CUBE_SIZE + 1, 1):
            (obstacle_x, obstacle_y) = related_to_world(x * grid.scale, y * grid.scale, cube.pose.rotation.angle_z.degrees)
            grid.addObstacle(
                position_to_grid(
                    grid,
                    cube.pose.position.x + obstacle_x,
                    cube.pose.position.y + obstacle_y,
                    grid_init_start_pose)
            )


def position_to_grid(grid: CozGrid, x, y, grid_init_start_pose):
    x = (int)(x)
    y = (int)(y)
    (init_x, init_y) = grid_init_start_pose
    result = ((int)(x / grid.scale + init_x), (int)(y / grid.scale + init_y))
    return result


def set_goal(grid: CozGrid, cube: cozmo.objects.LightCube, grid_init_start_pose):
    grid.clearGoals()
    goal_degrees = dict()

    # Cube right and back will be the picture, choose right this time
    (goal_x, goal_y) = related_to_world(- grid.scale * (CUBE_SIZE + 1), 0, cube.pose.rotation.angle_z.degrees) # Back
    goal_degree = cube.pose.rotation.angle_z.degrees
    (grid_x, grid_y) = position_to_grid(grid, cube.pose.position.x + goal_x, cube.pose.position.y + goal_y, grid_init_start_pose)
    if grid.coordInBounds((grid_x, grid_y)):
        goal_degrees[f"{int(grid_x)}, {int(grid_y)}"] = goal_degree
        grid.addGoal((grid_x, grid_y))

    (goal_x, goal_y) = related_to_world(0, - grid.scale * (CUBE_SIZE + 1), cube.pose.rotation.angle_z.degrees) # Right
    goal_degree = cube.pose.rotation.angle_z.degrees + 90
    (grid_x, grid_y) = position_to_grid(grid, cube.pose.position.x + goal_x, cube.pose.position.y + goal_y, grid_init_start_pose)
    if grid.coordInBounds((grid_x, grid_y)):
        goal_degrees[f"{int(grid_x)}, {int(grid_y)}"] = goal_degree
        grid.addGoal((grid_x, grid_y))

    return goal_degrees


def related_to_world(x, y, heading_deg):
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
