from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append("utils/")
import a_star
import fire_swarm

pygame.init()

def extract_colors(colors = []):
    color_vals = []
    for color in colors:
        [color_val] = [(c,v) for c,v in pygame.color.THECOLORS.items() if color == c]
        color_vals.append(color_val[1])
    if len(colors) == 1:
        return color_vals[0]
    else:
        return color_vals


# cell_colors = [v for c,v in pygame.color.THECOLORS.items() if 'light' in c]
robot_c = ['blue','orange','yellow','green','red','purple','violet','magenta','blue','orange']
robot_colors = extract_colors(['blue','orange','yellow','green','red','purple','violet','magenta','blue','orange'])
cell_colors = extract_colors(['lightblue','lightgoldenrod','lightyellow','lightgreen','lightcoral','lavender','lightsteelblue','lightpink','lightblue','lightgoldenrod'])

class Environment:
    def __init__(self,input_array) -> None:
        self.environment = input_array
        [self.map_width,self.map_height] = self.environment.shape
        self.scaling_factor = 700//self.map_height

    def draw_env(self,paths = []):
        path_points = []
        for path in paths:
            for point in path:
                path_points.append([round(point[0]),round(point[1])])
        count = 0
        self.surface = pygame.display.set_mode((self.map_width*self.scaling_factor,self.map_height*self.scaling_factor))
        self.surface.fill(extract_colors(['white']))
        for i in range(self.environment.shape[0]):
            for j in range(self.environment.shape[1]):
                if self.environment[i,j] == 0:
                    color = extract_colors(['gray30'])
                elif 200 > self.environment[i,j] >= 100:
                    color = cell_colors[int(self.environment[i,j])%10]
                if [i,j] in path_points:
                    color = extract_colors(['gray95'])
                if 400 > self.environment[i,j] >= 200:
                    index = int(self.environment[i,j] % 10)
                    color = robot_colors[index]
                pygame.draw.rect(self.surface, color, pygame.Rect(j*self.scaling_factor,(i)*self.scaling_factor,self.scaling_factor,self.scaling_factor))
        pygame.display.update()


FS = fire_swarm.Fire_Swarm(5,[50,50],10)
space = VC.voronoi_cells()
env = Environment(space)
log = []
t = 0
running = True
parking_lot = False

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    t+=1
    FS.detect_fire()
    FS.plan_path()
    for current_step in range(VC.steps_before_new_cells):
        
        
        for robot_id in range(VC.number_of_robots):
            # VC.individual_voronoi_path(robot_id)
            VC.robot_positions[robot_id] = VC.paths[robot_id][current_step]
            log.append([robot_id,t,VC.robot_positions[robot_id][0],VC.robot_positions[robot_id][1]])

    if len(VC.deadlocked_robots)>0:
        print([robot_c[i%10] for i in VC.deadlocked_robots])
    env.draw_env(VC.paths)
    pygame.display.update()

    space = VC.voronoi_cells()
    pygame.time.delay(100)
    if VC.success == 1:
        pygame.time.delay(1000)
        break
