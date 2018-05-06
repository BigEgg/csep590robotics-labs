from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
import heapq


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
    start_pose = grid.getStart()
    goal_pose = grid.getGoals()[0]

    frontier = PriorityQueue()
    frontier.put(start_pose, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start_pose] = None
    cost_so_far[start_pose] = 0

    while not frontier.empty():
        current = frontier.get()
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
                frontier.put(next_pose, priority)
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
    return abs(current_x - goal_x) + abs(current_y - goal_y)


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

    while not stopevent.is_set():
        pass  # Your code here


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
