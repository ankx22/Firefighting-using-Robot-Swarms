import bleach
import numpy as np
import time
import pygame
from sklearn.metrics import euclidean_distances
import wildfire_a_star
import wildfire_prm
import path_smoothener
from matplotlib import pyplot as plt

class mapenv:
    def __init__(self,map_width,map_height):
        self.groundcolor = (100,50,30)
        self.black = (20,20,20)
        self.red = (200,0,0)
        self.blue = (50,255,255)
        self.green = (0,255,50)
        self.bushgreen = (80,160,50)
        self.fireorange = (230,100,40)
        self.white = (0,0,0)
        self.grey = (120,120,120)
        self.map_width = map_width
        self.map_height = map_height
        self.collision = False
        self.scaling_factor = 10

    def draw_map(self,forest):
        self.surface = pygame.display.set_mode((self.map_width,self.map_height))
        self.surface.fill(self.groundcolor)
        for i in range(forest.shape[0]):
            for j in range(forest.shape[1]):
                if forest[i,j] == 1:        # background
                    color = self.groundcolor
                elif forest[i,j] == 0:      # bush
                    color = self.bushgreen
                elif forest[i,j] == 2:      # bush on fire
                    color = self.fireorange
                elif forest[i,j] == 4:      # bush on fire
                    color = self.black
                pygame.draw.rect(self.surface, color, pygame.Rect(i*self.scaling_factor,j*self.scaling_factor,self.scaling_factor,self.scaling_factor))
        return self.surface

    def col_det(self,car1_rect):
        for i in self.obstacles:
            if car1_rect.colliderect(i):
                self.collision = True
        return self.collision


def euc_dist(x,y,i,j):
    dist = np.sqrt((x-i)**2 + (y-j)**2)
    return dist

def set_fire(forest,timer,burning,ft_x,ft_y):
    timemult = 6
    simtime = timer * timemult
    burn_radius = 2
    extinguish_radius = 2
    if int(simtime) % 60 == 0:      # new fire
        x = 0
        y = 0
        while forest[x,y] != 0:
            x = np.random.randint(50)
            y = np.random.randint(50)
            burning.append([x,y,int(simtime)])
            time.sleep(1)
        forest[x,y] = 2
        print('Wumpus set fire at ', x*10, y*10,' at time',int(simtime))
        print(burning)
    
    for bush in burning:        # fire spreading and extinguishing
        if forest[bush[0],bush[1]] == 2:
            if euc_dist(bush[0],bush[1],ft_x,ft_y)<extinguish_radius and forest[i,j]==2:
                forest[i,j] = 4
                # print('Fire Extinguished')

            if simtime - bush[2] > 20:
                for i in range(forest.shape[0]):
                    for j in range(forest.shape[1]):
                        if euc_dist(bush[0],bush[1],i,j)<burn_radius and forest[i,j]==0:
                            forest[i,j] = 2
                            burning.append([i,j,int(simtime)])
                        else:
                            continue
                # print('Fire Spreads at time',int(simtime))
            # time.sleep(1)
    return forest , burning

def get_closest_burning(x,y,burning):
    min_dist = 420
    for bush in burning:
        dist = euc_dist(x,y,bush[0],bush[1])
        if dist<min_dist:
            closest_x = bush[0]
            closest_y = bush[1]
    return closest_x , closest_y

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


#####  MAIN  #####   MAIN  #####  MAIN  #####  MAIN  #####  MAIN  #####  MAIN  #####  MAIN  #####

forest = np.load('map1.npy')
forest = np.flip(forest,axis=1)
pygame.init()
car_im = pygame.image.load('ftruck5.png')
running = True
Simulation = True
while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # draw the map and environment
    burning = []
    in_x,in_y = 10,10
    env = mapenv(500,500)
    tic = time.time()
    ox,oy = wildfire_a_star.obstacles()
    rx,ry = [],[]
    oldlen = 0
    i = 0
    while Simulation:

        toc = time.time()
        forest , burning = set_fire(forest,toc-tic,burning,in_x,in_y)
        surface = env.draw_map(forest)
        if len(burning) > oldlen:
            gx,gy = get_closest_burning(in_x,in_y,burning)
            r1x,r1y = wildfire_a_star.main(in_x,in_y,gx,gy)
            rx.extend(np.flip(r1x))
            ry.extend(np.flip(r1y))
            xyt = path_smoothener.main(rx,ry)
        

        angle = -xyt[i,2]
        pivot = [xyt[i,0]*10, 500-xyt[i,1]*10]
        offset = pygame.math.Vector2(30, 0)
        i = i+1
        # Rotated version of the image and the shifted rect.
        car_im2, rect = rotate(car_im, angle, pivot, offset)
        pygame.draw.circle(surface, (30, 250, 70), pivot, 3)  # Pivot point.

        # Drawing.
        surface.blit(car_im2, rect)  # Blit the rotated image.


        pygame.display.update()

    # pygame.time.delay(1000)