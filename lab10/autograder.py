from grid import *
from visualizer import *
from planning import *
import sys
import threading
import json


# Thread to grade A* algorithm
class GradingThread(threading.Thread):

    def __init__(self, grid, solution):
        threading.Thread.__init__(self, daemon=True)
        self.grid = grid
        self.solution = solution

    def run(self):
        print("Grader running...\n")
        astar(self.grid, heuristic)
        points = 0
        
        if len(self.grid.getVisited()) <= self.solution['expanded'] and len(self.grid.getVisited()) > 0:
            print("Acceptable number of expanded nodes!")
            print("Expanded nodes: " + str(len(self.grid.getVisited())))
            print("1.0/1.0 points")
            points += 1.0
        elif len(self.grid.getVisited()) == 0:
            print("No expanded nodes!")
            print("Yours: " + str(len(self.grid.getVisited())))
            print("Solution: " + str(self.solution['expanded']))
            print("0/1.0 points")
        else:
            print("Too many expanded nodes!")
            print("Yours: " + str(len(self.grid.getVisited())))
            print("Solution: " + str(self.solution['expanded']))
            print("0/1.0 points")

        print()
        pathlen = self.grid.checkPath()
        if pathlen < 0:
            print("Path invalid! Intersects obstacles or has moves > 1 grid space.")
            print("0/2.0 points")
        elif pathlen == 0:
            print("There is no path! Make sure you're using grid.setPath")
            print("0/2.0 points")
        elif grid.getPath()[0] != grid.getStart():
            print("Path does not include start point!")
            print("0/2.0 points")
        elif grid.getPath()[-1] not in grid.getGoals():
            print("Path does not reach goal!")
            print("0/2.0 points")
        elif pathlen <= self.solution['pathlen']:
            print("Correct path length!")
            print("Path length: " + str(pathlen))
            print("2.0/2.0 points")
            points += 2.0
        else:
            print("Incorrect path length!")
            print("Yours: " + str(pathlen))
            print("Solution: " + str(self.solution['pathlen']))
            print("0/2.0 points")
            
        print("\nScore = " + str(points) + "/3.0\n")

        print("Close visualizer window to exit")


if __name__ == "__main__":
    test = {}
    if len(sys.argv) > 1:
        try:
            with open(sys.argv[1]) as testfile:
                test = json.loads(testfile.read())
        except:
            print("Error opening test file, please check filename and json format")
            raise
    else:
        print("correct usage: python3 autograder.py <testfile>")
        exit()
            
    grid = CozGrid(test['mapfile'])
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    grader = GradingThread(grid, test['solution'])
    grader.start()
    visualizer.start()
