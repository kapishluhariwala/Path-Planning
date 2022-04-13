import math
import random
import pygame
from environment import Colors

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

class RRT:
    def __init__(self, start, goal, dimensions, obstacles):
        (x,y) = start
        self.start = start
        self.goal = goal
        self.mapW, self.mapH = dimensions
        self.goalFlag = False

        self.X = []
        self.Y = []
        self.parent = []

        self.X.append(x)
        self.Y.append(y)
        self.parent.append(0)

        self.obstacles = obstacles

        self.goalState = None
        self.path = []
        self.iteration = 0

    def draw(self, map, survalance_path, i):
        map.fill(type_color.get(Colors.DEFAULT))

        for _, path in enumerate(survalance_path):
            pygame.draw.circle(map, type_color.get(Colors.PATH), path, 5, 0)
            if _ == i:
                pygame.draw.circle(map, type_color.get(Colors.CAR), path, 5, 0)

        pygame.draw.circle(map, type_color.get(Colors.START), self.start, 5, 0)
        pygame.draw.circle(map, type_color.get(Colors.GOAL), self.goal, 10, 2)
        for obs in self.obstacles:
            rect = pygame.Rect((obs[0],obs[1]), (obs[2],obs[3]))
            pygame.draw.rect(map,  type_color.get(Colors.WALL), rect, 3)

        if(self.goalFlag):
            for node in self.getPathCoords():
                pygame.draw.circle(map, type_color.get(Colors.PATH), node, 5, 0)

    def addNode(self, n, x, y):
        self.X.insert(n,x)
        self.Y.insert(n,y)

    def removeNode(self, n):
        self.X.pop(n)
        self.Y.pop(n)

    def addEdge(self, parent, child):
        self.parent.insert(child, parent)

    def removeEdge(self, n):
        self.parent.pop(n)

    def numberOfNodes(self):
        return len(self.X)

    def distance(self, n1, n2):
        (x1, y1) = (self.X[n1], self.Y[n1])
        (x2, y2) = (self.X[n2], self.Y[n2])
        px = (float(x1) - float(x2))**2
        py = (float(y1) - float(y2))**2
        return math.sqrt(px + py)

    def sample_envir(self):
        x = int(random.uniform(0, self.mapW))
        y = int(random.uniform(0, self.mapH))
        return x,y

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def isFree(self):
        n = self.numberOfNodes() - 1
        (x,y) = (self.X[n], self.Y[n])
        obs = self.obstacles.copy()
        while len(obs)>0:
            rectangle = obs.pop(0)
            if (x>=rectangle[0] and y>=rectangle[1]) and (x<=rectangle[0]+rectangle[2] and y<=rectangle[1]+rectangle[3]):
                self.removeNode(n)
                return False
        return True

    def crossObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()
        while len(obs)>0:
            rectangle = obs.pop(0)
            for i in range(0,101):
                u = i/100
                x = x1*u + x2*(1-u)
                y = y1*u + y2*(1-u)
                if (x>=rectangle[0] and y>=rectangle[1]) and (x<=rectangle[0]+rectangle[2] and y<=rectangle[1]+rectangle[3]):
                    return True

        return False

    def connect(self, n1, n2):
        (x1,y1) = (self.X[n1], self.Y[n1])
        (x2,y2) = (self.X[n2], self.Y[n2])
        if self.crossObstacle(x1, x2, y1, y2):
            self.removeNode(n2)
            return False
        else:
            self.addEdge(n1,n2)
            return True

    def step(self, nnear, nrand, dmax = 35):
        d = self.distance(nnear, nrand)
        if d > dmax:
            u = dmax/d
            (xnear, ynear) = (self.X[nnear], self.Y[nnear])
            (xrand, yrand) = (self.X[nrand], self.Y[nrand])
            (px, py) = (xrand-xnear, yrand-ynear)
            theta = math.atan2(py,px)
            (x,y) = (int(xnear+dmax*math.cos(theta)), int(ynear+dmax*math.sin(theta)))
            self.removeNode(nrand)
            if abs(x-self.goal[0])<dmax and abs(y-self.goal[1])<dmax:
                self.addNode(nrand, self.goal[0], self.goal[1])
                self.goalState = nrand
                self.goalFlag = True
            else:
                self.addNode(nrand, x, y)

    def pathToGoal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalState)
            newpos = self.parent[self.goalState]
            while newpos!=0:
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x,y = self.X[node], self.Y[node]
            pathCoords.append((x,y))
        return pathCoords

    def bias(self, ngoal):
        n = self.numberOfNodes()
        self.addNode(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.X,self.Y,self.parent

    def expand(self):
        n = self.numberOfNodes()
        x, y = self.sample_envir()
        self.addNode(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.X,self.Y,self.parent

    def findPath(self, map, survalance_path, i):
        if not self.pathToGoal():
            if self.iteration % 10 == 0:
                X, Y, parent = self.bias(self.goal)
                pygame.draw.circle(map, type_color.get(Colors.NODE), (X[-1], Y[-1]), 4, 0)
                pygame.draw.line(map, type_color.get(Colors.NODE), (X[-1], Y[-1]), (X[parent[-1]], Y[parent[-1]]), 1)
            else:
                X, Y, parent = self.expand()
                pygame.draw.circle(map, type_color.get(Colors.NODE), (X[-1], Y[-1]), 4, 0)
                pygame.draw.line(map, type_color.get(Colors.NODE), (X[-1], Y[-1]), (X[parent[-1]], Y[parent[-1]]), 1)

            self.iteration += 1
            return False
        return True
