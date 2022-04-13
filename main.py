import pygame
import pygame_gui
from environment import buildEnvironment
from astar import Astar
from rrt import RRT
from djikstra import Djikstra
import sys
import time

def main(start, goal):
    dimensions = (1200,600)
    blockSize = 10
    # start = (50,50)
    # goal = (1100,570)
    obsDim = (30,30)
    obsNum = 80
    hasPathFound = False
    simulate = False
    animate = False
    i=0
    survalance_path = []

    env = buildEnvironment(start, goal, dimensions, obsDim, obsNum, blockSize)
    env.generateObs()

    graph = Astar(start, goal, dimensions, blockSize, env.obstacles)
    graph.draw(env.map, survalance_path, i)

    running = True
    start_time = time.time()
    while running:
        current_time = time.time()
        elapsed_time = current_time - start_time

        time_delta = env.clock.tick(60)/1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.display.quit()
                pygame.quit()
                sys.exit()
            env.manager.process_events(event)
            if not simulate:
                if pygame.mouse.get_pressed()[2]:
                    goal = pygame.mouse.get_pos()
                    goal = (round(goal[0]/10)*10, round(goal[1]/10)*10)
                    graph = Astar(start, goal, dimensions, blockSize, env.obstacles)
                    graph.draw(env.map, survalance_path, i)

                if pygame.mouse.get_pressed()[1]:
                    point = pygame.mouse.get_pos()
                    point = (round(point[0]/10)*10, round(point[1]/10)*10)
                    if point not in survalance_path:
                        survalance_path.append(point)

                    graph.draw(env.map, survalance_path, i)


            if event.type == pygame_gui.UI_BUTTON_PRESSED:
                if event.ui_element == env.button:
                    main(start, goal)
                if event.ui_element == env.simulate:
                    start_time = time.time()
                    simulate = True
                    hasPathFound = False
                if event.ui_element == env.rrtButton:
                    graph = RRT(start, goal, dimensions, env.obstacles)
                    graph.draw(env.map,survalance_path, i)
                if event.ui_element == env.astarButton:
                    graph = Astar(start, goal, dimensions, blockSize, env.obstacles)
                    graph.draw(env.map,survalance_path, i)
                if event.ui_element == env.djikstraButton:
                    graph = Djikstra(start, goal, dimensions, blockSize, env.obstacles)
                    graph.draw(env.map,survalance_path, i)

        if not hasPathFound and simulate:
            hasPathFound = graph.findPath(env.map,survalance_path, i)
            finishedTime = elapsed_time

        if hasPathFound:
            env._display_surf.fill((255,255,255))
            simulate = False
            env.updateTime(int(finishedTime))
            graph.draw(env.map,survalance_path, i)
            animate = True
            i=i+1
            if i>=len(survalance_path):
                i=0


        env.manager.draw_ui(env._display_surf)
        env.manager.update(time_delta)
        pygame.display.update()

if __name__ == '__main__':
    start = (50,50)
    goal = (1100,570)
    main(start, goal)
