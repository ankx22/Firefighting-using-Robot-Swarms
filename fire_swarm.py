import numpy as np
from numpy.linalg import norm
import sys
sys.path.append("utils/")
import a_star
import obstacle_field_gen
from matplotlib import pyplot as plt

def euclidian_dist(p1,p2):
    dist = ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 ) ** 0.5
    return dist

class Fire_Swarm:
    def __init__(self, N_robots, dimensions, desired_density):
        
        self.length,self.width = dimensions
        self.obstacle_density = desired_density
        self.number_of_robots = N_robots
        self.steps_before_new_cells = 1

        self.space,_,_ = obstacle_field_gen.main(self.length,self.width,self.obstacle_density,borders = True)

        self.robot_positions = self.valid_points()                # random test case
        self.goal_positions = self.valid_points()

        self.segmented_space = self.space
        self.past_positions = []
        self.deadlocked_robots = []
        self.deadlock_threshhold = 10
        self.nodecount = 0

        self.reached = [int(self.robot_positions[robot] == self.goal_positions[robot]) for robot in range(self.number_of_robots) ]
        self.success = 0

        # self.vanilla_collisions = 0
        # self.collisons_per_step = 0
        # for i in range(self.number_of_robots):
        #     self.vanilla_collisions += obstacles_on_line(self.robot_positions[i],self.goal_positions[i],self.space)
        #     self.collisons_per_step += obstacles_on_line(self.robot_positions[i],self.goal_positions[i],self.space)/max(1,euclidian_dist(self.robot_positions[i],self.goal_positions[i]))
        # print('\n',self.vanilla_collisions)


    def valid_points(self,n=2):
        ''' returns a list of lists where there are no obstacles, [[x1,y1],[x2,y2],[x3,y3]....] '''
        points = []
        for robot in range(self.number_of_robots):
            i,j = np.random.randint(self.length),np.random.randint(self.width)
            while self.space[i,j] != 1:
                i,j = np.random.randint(self.length),np.random.randint(self.width)
            if n == 3:
                points.append([i,j,np.random.randint(0,360)])
            else:
                points.append([i,j])
        return points