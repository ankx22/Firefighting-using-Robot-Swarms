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

# # cell_colors = [v for c,v in pygame.color.THECOLORS.items() if 'light' in c]
# robot_c = ['blue','orange','yellow','green','red','purple','violet','magenta','blue','orange']
robot_colors = extract_colors(['blue','orange','yellow','green','red','purple','violet','magenta','blue','orange'])
# cell_colors = extract_colors(['lightblue','lightgoldenrod','lightyellow','lightgreen','lightcoral','lavender','lightsteelblue','lightpink','lightblue','lightgoldenrod'])

class Environment:
    def __init__(self,input_array) -> None:
        self.environment = input_array
        [self.map_width,self.map_height] = self.environment.shape
        self.scaling_factor = 700//self.map_height

    def draw_env(self,FS):
        path_points = []
        for path in FS.paths:
            for point in path[1:]:
                path_points.append([round(point[0]),round(point[1])])
        count = 0
        self.surface = pygame.display.set_mode((self.map_width*self.scaling_factor,self.map_height*self.scaling_factor))
        self.surface.fill(extract_colors(['white']))
        for i in range(self.environment.shape[0]):
            for j in range(self.environment.shape[1]):
                if self.environment[i,j] == 1:    # Ground
                    color = extract_colors(['gray80'])
                if [i,j] in path_points:
                    color = extract_colors(['gray85'])    
                if self.environment[i,j] == 0:      # Tree
                    color = extract_colors(['forestgreen'])
                elif self.environment[i,j] == 2:    # Reservoir
                    color = extract_colors(['darkblue'])
                elif self.environment[i,j] == 3:    # Ash
                    color = extract_colors(['gray30'])
                elif self.environment[i,j] == 10:   # ext
                    color = extract_colors(['yellow'])
                elif self.environment[i,j] > 10:   # Fire
                    color = extract_colors(['orange'])
                if [i,j] in FS.robot_positions:
                    index = FS.robot_positions.index([i,j])
                    if FS.water[index]:
                        color = extract_colors(['red'])
                    elif not FS.water[index]:
                        color = extract_colors(['darkred'])
                pygame.draw.rect(self.surface, color, pygame.Rect(j*self.scaling_factor,(i)*self.scaling_factor,self.scaling_factor,self.scaling_factor))
        pygame.display.update()


FS = fire_swarm.Fire_Swarm(15,[50,50],10)
env = Environment(FS.simulated_space)
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            print(FS.time_steps)
            running = False
    
    FS.step()

    env.draw_env(FS)
    pygame.display.update()

    # pygame.time.delay(100)