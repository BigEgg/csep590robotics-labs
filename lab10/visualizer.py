from tkinter import *
from grid import *
import threading

class Visualizer():
    """Visualizer to display status of an associated CozGrid instance, supplied on instantiation

        Should be started in main thread to avoid issue of GUI code not working
        in spawned threads on OSX
    """
        

    def __init__(self, grid, scale=25):
        self.grid = grid
        self.running = threading.Event()
        self.scale = scale


    def drawgrid(self):
        """Draw grid lines
        """
        
        self.canvas.create_rectangle(0, 0, self.grid.width * self.scale, self.grid.height * self.scale)
        for y in range(1,self.grid.height):
            self.canvas.create_line(0, y * self.scale, int(self.canvas.cget("width")), y * self.scale)
        for x in range(1,self.grid.width):
            self.canvas.create_line(x * self.scale, 0, x * self.scale, int(self.canvas.cget("height")))


    def colorsquare(self, location, color, bg=False, tags=''):
        """Draw a colored square at a given location

            Arguments:
            location -- coordinates of square
            color -- desired color, hexadecimal string (e.g.: '#C0FFEE')
            bg -- draw square in background, default False
            tags -- tags to apply to square, list of strings or string
        """
        coords = (location[0]*self.scale, (self.grid.height - 1 - location[1])*self.scale)
        rect = self.canvas.create_rectangle(coords[0], coords[1], coords[0] + self.scale, coords[1] + self.scale, fill=color, tags=tags)
        if bg:
            self.canvas.tag_lower(rect)


    def drawstart(self):
        """Redraw start square
            Color is green by default
        """
        self.canvas.delete('start')
        if self.grid._start != None:
            self.colorsquare(self.grid._start, '#00DD00', tags='start')


    def drawgoals(self):
        """Redraw all goal cells
            Color is blue by default
        """
        self.canvas.delete('goal')
        for goal in self.grid._goals:
            self.colorsquare(goal, '#0000DD', tags='goal')


    def drawallvisited(self):
        """Redraw all visited cells
            Color is light gray by default
        """
        
        self.canvas.delete('visited')
        for loc in self.grid._visited:
            self.colorsquare(loc, '#CCCCCC', bg = True, tags='visited')


    def drawnewvisited(self):
        """Draw any new visited cells added since last call
            Does not delete previously drawn visited cells
            Color is light gray by default
        """
        
        for loc in self.grid._newvisited:
            self.colorsquare(loc, '#CCCCCC', bg = True, tags='visited')
        self.grid._newvisited = []


    def drawobstacles(self):
        """Redraw all obstacles
            Color is dark gray by default
        """
        
        self.canvas.delete('obstacle')
        for obstacle in self.grid._obstacles:
            self.colorsquare(obstacle, '#222222', bg = True, tags='obstacle')


    def drawpathedge(self, start, end):
        """Draw a path segment between two cells

            Arguments:
            start -- starting coordinate
            end -- end coordinate
        """
        
        startcoords = ((start[0] + 0.5) * self.scale, (self.grid.height - (start[1] + 0.5)) * self.scale)
        endcoords = ((end[0] + 0.5) * self.scale, (self.grid.height - (end[1] + 0.5)) * self.scale)
        self.canvas.create_line(startcoords[0], startcoords[1], endcoords[0], endcoords[1], fill = '#DD0000', width = 5, arrow = LAST, tag='path')


    def drawpath(self):
        """Draw the grid's path, if any
        """
        
        self.canvas.delete('path')
        if len(self.grid._path) > 1:
            current = self.grid._path[0]
            for point in self.grid._path[1:]:
                self.drawpathedge(current, point)
                current = point


    def update(self, *args):
        """Redraw any updated grid elements
        """
        
        self.grid.lock.acquire()
        self.grid.updated.clear()
        
        if 'path' in self.grid.changes:
            self.drawpath()
        if 'visited' in self.grid.changes:
            self.drawnewvisited()
        if 'allvisited' in self.grid.changes:
            self.drawallvisited()
        if 'goals' in self.grid.changes:
            self.drawgoals()
        if 'start' in self.grid.changes:
            self.drawstart()
        if 'obstacles' in self.grid.changes:
            self.drawobstacles()

        self.grid.changes = []
        self.grid.lock.release()


    def setup(self):
        """Do initial drawing of grid, start, goals, and obstacles
        """
        
        self.grid.lock.acquire()
        
        self.drawgrid()
        self.drawgoals()
        self.drawstart()
        self.drawobstacles()
        
        self.grid.lock.release()
            

    def start(self):
        """Start the visualizer, must be done in main thread to avoid issues on macs
            Blocks until spawned window is closed
        """
        
        # Set up Tk stuff
        master = Tk()
        self.canvas = Canvas(master, width = self.grid.width * self.scale, height = self.grid.height * self.scale, bd = 0, bg = '#FFFFFF')
        self.canvas.bind('<<Update>>', self.update)
        self.canvas.pack()

        # Draw grid and any initial items
        self.setup()

        # Start mainloop and indicate that it is running
        self.running.set()
        mainloop()

        # Indicate that main loop has finished
        self.running.clear()

    def trig_update(self):
        """Trigger an update event, should make things more thread-safe
        """
        self.canvas.event_generate('<<Update>>')


class UpdateThread(threading.Thread):
    """Thread to update a visualizer instance whenever its associated CozGrid instance is modified

        Arguments:
        visualizer -- visualizer to monitor
    """    
    
    def __init__(self, visualizer):
        threading.Thread.__init__(self, daemon=True)
        self.visualizer = visualizer

    def run(self):
        while threading.main_thread().is_alive():
            self.visualizer.grid.updated.wait()
            if self.visualizer.running.is_set():
                self.visualizer.trig_update()
