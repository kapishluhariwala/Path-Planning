import pygame
import random
from environment import Colors
import math

type_color = {
    Colors.DEFAULT: [255, 255, 255],
    Colors.WALL: [0, 0, 0],
    Colors.START: [255, 165, 0],
    Colors.GOAL: [127, 0, 255],
    Colors.OPEN: [0, 255, 0],
    Colors.CLOSED: [255, 0, 0],
    Colors.PATH: [39, 41, 80],
    Colors.CAR: [255, 255, 0],
}

class Block:
    def __init__(self, row, col, width):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.width = width
        self.type = Colors.DEFAULT

        self.gCost = 0
        self.hCost = 0
        self.parent = None

        self.rect = pygame.Rect(self.x, self.y, width, width)

    def draw(self, win, color):
        pygame.draw.rect(win, color, self.rect)

    def set_wall(self):
        self.type = Colors.WALL

    def set_start(self):
        self.type = Colors.START

    def set_end(self):
        self.type = Colors.GOAL

    def set_closed(self):
        self.type = Colors.CLOSED

    def set_path(self):
        self.type = Colors.PATH

    def set_animate(self):
        self.type = Colors.CAR

    def set_open(self):
        self.type = Colors.OPEN

    def f_cost(self):
        return self.gCost + self.hCost

class Astar:
    def __init__(self, start, goal, dimensions, blockSize, obstacles):
        self.start = start
        self.goal = goal
        self.mapW, self.mapH = dimensions
        self.rows, self.cols = self.mapW//blockSize, self.mapH//blockSize
        self.blockSize = blockSize

        self.blocks = self.createBlocks(obstacles)
        self.blocks[start[1]//blockSize][start[0]//blockSize].set_start()
        self.blocks[goal[1]//blockSize][goal[0]//blockSize].set_end()

        self.start_block = self.blocks[self.start[1]//self.blockSize][self.start[0]//self.blockSize]
        self.end_block = self.blocks[self.goal[1]//self.blockSize][self.goal[0]//self.blockSize]
        self.open_list = []
        self.closed_list = []
        self.open_list.append(self.start_block)

    def draw(self, map, survalance_path, i):
        map.fill(type_color.get(Colors.DEFAULT))
        for _, path in enumerate(survalance_path):
            self.blocks[path[1]//self.blockSize][path[0]//self.blockSize].set_path()
            if _ == i:
                self.blocks[survalance_path[i][1]//self.blockSize][survalance_path[i][0]//self.blockSize].set_animate()

        for col in range(len(self.blocks)):
            for row in range(len(self.blocks[0])):
                self.blocks[col][row].draw(map, type_color.get(self.blocks[col][row].type))

        for x in range(0, self.mapW, self.blockSize):
            pygame.draw.line(map, (125,125,125), (x, 0), (x, self.mapH))
            for y in range(0, self.mapH, self.blockSize):
                pygame.draw.line(map, (125,125,125), (0, y), (self.mapW, y))
        pygame.draw.line(map, (125,125,125), (self.mapW, 0), (self.mapW, self.mapH))
        pygame.display.update()

    def createBlocks(self, obstacles):
        blocks = []
        for y in range(self.cols):
            row = []
            for x in range(self.rows):
                item = Block(x, y, self.blockSize)
                row.append(item)
            blocks.append(row)
        for obs in obstacles:
            for x in range(obs[1]//self.blockSize, ((obs[1]+obs[3])//self.blockSize)):
                for y in range(obs[0]//self.blockSize, ((obs[0]+obs[2])//self.blockSize)):
                    blocks[x][y].set_wall()
        return blocks

    def get_distance(self,block_a, block_b):
        dist_x = abs(block_a.x - block_b.x)
        dist_y = abs(block_a.y - block_b.y)
        # if dist_x > dist_y:
        #     return 14*dist_y + 10*(dist_x-dist_y)
        # return 14 * dist_x + 10 * (dist_y - dist_x)
        return dist_x + dist_y

    def retrace_path(self, start, end):
        path = []
        current_block = end
        while not current_block == start:
            path.append(current_block)
            current_block = current_block.parent
        path.append(start)
        path.reverse()
        for block in path:
            block.set_path()

    def get_neighbours(self, block):
        neighbours = []
        for x in range(-1, 2):
            for y in range(-1, 2):
                if x == 0 and y == 0:
                    continue
                else:
                    check_x = block.row + x
                    check_y = block.col + y
                    if check_x >= 0 and check_x < self.rows and check_y >= 0 and check_y < self.cols:
                        neighbours.append(self.blocks[check_y][check_x])
        return neighbours

    def findPath(self, map,survalance_path, i):
        if(len(self.open_list) <= 0):
            return False
        current_block = self.open_list[0]
        for i in range(1, len(self.open_list)):
            if self.open_list[i].f_cost() <= current_block.f_cost() or self.open_list[i].f_cost() == current_block.f_cost() and self.open_list[i].hCost < current_block.hCost:
                current_block = self.open_list[i]
        self.open_list.remove(current_block)
        self.closed_list.append(current_block)
        current_block.set_closed()

        self.draw(map,survalance_path, i)

        if current_block == self.end_block:
            self.retrace_path(self.start_block, self.end_block)
            return True
        for neighbour in self.get_neighbours(current_block):
            if neighbour.type == Colors.WALL or neighbour in self.closed_list:
                continue
            new_cost_to_neighbour = current_block.gCost + self.get_distance(current_block, neighbour)
            if new_cost_to_neighbour < neighbour.gCost or neighbour not in self.open_list:
                neighbour.gCost = new_cost_to_neighbour
                neighbour.hCost = self.get_distance(neighbour, self.end_block)
                neighbour.parent = current_block
                if neighbour not in self.open_list:
                    self.open_list.append(neighbour)
                    neighbour.set_open()
