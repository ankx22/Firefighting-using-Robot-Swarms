from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import Voronoi
# import Voronoi_V2
import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append("utils/")
import a_star

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




def rotate(surface, angle, pivot, offset):
    """Rotate the surface around the pivot point.
        surface (pygame.Surface): The surface that is to be rotated.
        angle (float): Rotate by this angle.
        pivot (tuple, list, pygame.math.Vector2): The pivot point.
        offset (pygame.math.Vector2): This vector is added to the pivot.
    """
    rotated_image = pygame.transform.rotozoom(surface, -angle, 1)  # Rotate the image.
    rotated_offset = offset.rotate(angle)  # Rotate the offset vector.
    # Add the offset vector to the center/pivot point to shift the rect.
    rect = rotated_image.get_rect(center=pivot+rotated_offset)
    return rotated_image, rect  # Return the rotated image and shifted rect.


def car_plot(surface,position):
    image = pygame.image.load('Car.png')
    car_im = pygame.transform.scale(image, (20, 50))
    angle_rad = -position[2]
    angle = angle_rad*180/3.14+180
    pivot = [14*position[1], 14*position[0]]
    offset = pygame.math.Vector2(0, 0)
    car_im2, rect = rotate(car_im, angle, pivot, offset)
    surface.blit(car_im2, rect)


VC = Voronoi.Voronoi_Cells(5,[50,50],10)
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

    VC.voronoi_paths()
    VC.detect_deadlock()
    for current_step in range(VC.steps_before_new_cells):
        t+=1
        
        for robot_id in range(VC.number_of_robots):
            # VC.individual_voronoi_path(robot_id)
            VC.robot_positions[robot_id] = VC.paths[robot_id][current_step]
            log.append([robot_id,t,VC.robot_positions[robot_id][0],VC.robot_positions[robot_id][1]])

    if len(VC.deadlocked_robots)>0:
        print([robot_c[i%10] for i in VC.deadlocked_robots])
    env.draw_env(VC.paths)
    if parking_lot:
        for robot_id,robot in enumerate(VC.robot_positions):
            car_plot(env.surface,VC.robot_positions[robot_id])
            # pygame.draw.rect(env.surface,(0,255,255),(611,473,100,50))
    pygame.display.update()
            # continue

    space = VC.voronoi_cells()
    pygame.time.delay(100)
    if VC.success == 1:
        # print(VC.nodecount)
        pygame.time.delay(1000)
        break

# np.save('map_paper_10.npy',VC.space)

linecolor = ['blue','orange','gold','lightgreen','red','purple','violet','magenta','blue','orange']
log = np.array(log)
log = log[np.argsort(log[:,0])]
individual_logs = np.split(log, np.where(np.diff(log[:,0]))[0]+1)
for i,path in enumerate(individual_logs):
    # print('\n',path)
    path = path[np.argsort(path[:,1])]
    plt.plot(path[:,3],50-path[:,2], linecolor[i%10])

ox,oy = a_star.obstacles(VC.space)
plt.plot(np.array(oy),50-np.array(ox),'.k')
plt.axis("equal")
plt.show()