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
        self.time_steps = 0

        self.space,_,_ = obstacle_field_gen.main(self.length,self.width,self.obstacle_density,borders = True)   # consists of 1 = blank and 0 = obstacles
        self.simulated_space = self.space   # contains information about fires and reservoir
        self.simulated_space[:3,:3] = 10    # 2 = reservoir, 11 to inf = fire

        self.robot_positions = self.valid_points(self.number_of_robots)                # random test case
        self.goal_positions = self.valid_points(self.number_of_robots)

        self.fires = []
        self.detected_fires = []

        self.fire_detection_radius = 10
        self.fire_figting_radius = 2
        self.buckets_per_fire = 3
        self.time_steps_before_ash = 60
        

    def valid_points(self,n=1):
        ''' returns a list of lists where there are no obstacles, [[x1,y1],[x2,y2],[x3,y3]....] '''
        points = []
        for robot in range(n):
            i,j = np.random.randint(self.length),np.random.randint(self.width)
            while self.space[i,j] != 1:
                i,j = np.random.randint(self.length),np.random.randint(self.width)
        if n==1:
            return [i,j]
        else:
            points.append([i,j])
            return points
        

    def generate_paths(self):
        self.assign_goal()
        self.paths = []
        for robot_id in range(self.number_of_robots):
            path = a_star.main(self.space,self.robot_positions[robot_id],self.goal_positions[robot_id], 
                            collisions = [self.robot_positions[:robot_id],self.robot_positions[robot_id+1:]])
            
            if path is None:        # if path is not foung, stay at current location
                path = [self.robot_positions[robot_id],self.robot_positions[robot_id]]
            self.paths.append(path)


    def start_fire(self):
        i,j = np.random.randint(self.length),np.random.randint(self.width)
        while self.simulated_space[i,j] != 0:
            i,j = np.random.randint(self.length),np.random.randint(self.width)
        print('Fire Started at ',[i,j])
        self.simulated_space[i,j] = 10 + self.buckets_per_fire
        self.fires.append([i,j])


    def detect_fire(self):
        for robot_id in range(self.number_of_robots):
            [x,y] = self.robot_positions[robot_id]
            observed_area = self.space[x-self.fire_detection_radius:x+self.fire_detection_radius+1,y-self.fire_detection_radius:y+self.fire_detection_radius+1]
            f = np.where(observed_area>10)
            fires = list(zip(f[0],f[1]))
            if type(fires[0])==list:
                for fire in fires:
                    self.detected_fires.append(fire)
                    print('Fire detected at ',fire)
            elif type(fires)==list:
                self.detected_fires.append(fires)
                print('Fire detected at, ',fires)


    def assign_goal(self,robot_id):
        if len(self.detected_fires>0):
            closest = np.inf
            for fire in self.detected_fires:
                d = euclidian_dist(self.robot_positions[robot_id],fire)
                if d < closest:
                    closest = d
                    temp_goal = fire
            self.goal_positions[robot_id] = temp_goal
        elif self.robot_positions[robot_id] == self.goal_positions[robot_id]:
            self.goal_positions[robot_id] = self.valid_points()


    def move(self):
        for robot_id in range(self.number_of_robots):
            self.robot_positions = self.paths[robot_id][1]


    def step(self):
        self.time_steps += 1
        self.move()
        self.detect_fire()
        self.generate_paths()

FS = Fire_Swarm(4,[50,50],10)
