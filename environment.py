import pygame
import random
import pygame_gui
from enum import Enum

class Colors(Enum):
    DEFAULT = 1
    WALL = 2
    START = 3
    GOAL = 4
    OPEN = 5
    CLOSED = 6
    PATH = 7
    NODE = 8
    CAR = 9

type_color = {
    Colors.DEFAULT: [255, 255, 255],
    Colors.WALL: [0, 0, 0],
    Colors.START: [255, 165, 0],
    Colors.GOAL: [127, 0, 255],
    Colors.OPEN: [0, 255, 0],
    Colors.CLOSED: [255, 0, 0],
    Colors.PATH: [39, 41, 80],
    Colors.NODE: [125, 125, 125],
    Colors.CAR: [255, 255, 0],
}

class buildEnvironment:
    def __init__(self, start, goal, dimensions, obsDim, obsNum, blockSize):
        self.start = start
        self.goal = goal
        self.blockSize = blockSize
        self.mapW, self.mapH = dimensions
        self.rows, self.cols = self.mapW//blockSize, self.mapH//blockSize

        pygame.init()
        pygame.display.set_caption('Path Planning')
        self._display_surf = pygame.display.set_mode((self.mapW + 200, self.mapH))
        self._display_surf.fill(type_color.get(Colors.DEFAULT))

        self.map = pygame.Surface.subsurface(self._display_surf, ((0,0),(self.mapW, self.mapH)))
        self.manager = pygame_gui.UIManager((self.mapW + 200, self.mapH), "theme.json")
        self.clock = pygame.time.Clock()

        self.obstacles = []
        self.obsDim = obsDim
        self.obsNum = obsNum

        self.astarButton = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((self.mapW + 50, 75), (100, 25)), text='Astar', manager=self.manager)
        self.djikstraButton = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((self.mapW + 50, 100), (100, 25)), text='Djikstra', manager=self.manager)
        self.rrtButton = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((self.mapW + 50, 125), (100, 25)), text='RRT', manager=self.manager)

        self.button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((self.mapW + 50, 275), (100, 50)), text='Generate', manager=self.manager)
        self.timetaken = pygame_gui.elements.UILabel(relative_rect=pygame.Rect((self.mapW + 50, 375), (100, 50)),text='Time Taken',manager=self.manager)
        self.time = pygame_gui.elements.UILabel(relative_rect=pygame.Rect((self.mapW + 50, 400), (100, 50)),text='',manager=self.manager)
        self.simulate = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((self.mapW + 50, 475), (100, 50)), text='Simulate', manager=self.manager)

    def updateTime(self, time):
        # self.time = pygame_gui.elements.UILabel(relative_rect=pygame.Rect((self.mapW + 50, 400), (100, 50)),text='',manager=self.manager)
        self.time.set_text(f'{time}')

    def makeRandomRect(self):
        uppercornerx = int(random.uniform(0, self.mapW - self.obsDim[0]))
        uppercornery = int(random.uniform(0, self.mapH - self.obsDim[1]))
        return (uppercornerx, uppercornery)

    def generateObs(self):
        for i in range(0, self.obsNum):
            collision = True
            while collision:
                pos = self.makeRandomRect()
                size = (int(random.uniform(10, self.obsDim[0])), int(random.uniform(10, self.obsDim[1])))
                collision = False
                if(self.checkCollision(pos, size)):
                    collision = True
            self.obstacles.append([pos[0], pos[1], size[0], size[1]])
        # print(self.obstacles)

    def checkCollision(self, pos, size):
        # x,y,width,height
        if(pos[0]<=self.start[0]-20 and pos[1]<=self.start[1]-20 and pos[0]+size[0]>=self.start[0]+10 and pos[1]+size[1]>=self.start[1]+20):
            return True
        if(pos[0]<=self.goal[0]-20 and pos[1]<=self.goal[1]-20 and pos[0]+size[0]>=self.goal[0]+20 and pos[1]+size[1]>=self.goal[1]+20):
            return True
        for obs in self.obstacles:
            if(pos[0]>=obs[0]-20 and pos[1]>=obs[1]-20 and pos[0]<=obs[0]+obs[2]+20 and pos[1]<=obs[1]+obs[3]+20):
                return True
            if(pos[0]+size[0]>=obs[0]-20 and pos[1]+size[1]>=obs[1]-20 and pos[0]+size[0]<=obs[0]+obs[2]+20 and pos[1]+size[1]<=obs[1]+obs[3]+10):
                return True
        return False
