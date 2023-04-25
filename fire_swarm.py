import numpy as np
from numpy.linalg import norm
import sys
sys.path.append("utils/")
import a_star
import obstacle_field_gen
from matplotlib import pyplot as plt
import random

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

        self.space,_,_ = obstacle_field_gen.main(self.length,self.width,self.obstacle_density,borders = False)   # consists of 1 = blank and 0 = obstacles
        self.simulated_space = self.space  # contains information about fires and reservoir
        self.forest = self.space.copy()

        self.res1 = self.valid_points()
        self.res2 = self.valid_points()
        print('R Reservoirs are at',self.res1,self.res2)
        self.simulated_space[self.res1[0]-1:self.res1[0]+2,self.res1[1]-1:self.res1[1]+2] = 2    # 2 = reservoir, 3 = ash, 11 to inf = fire
        self.simulated_space[self.res2[0]-1:self.res2[0]+2,self.res2[1]-1:self.res2[1]+2] = 2

        self.robot_positions = self.valid_points(self.number_of_robots)                # random test case
        self.goal_positions = self.valid_points(self.number_of_robots)
        self.water = [True for robot_id in range(self.number_of_robots)]

        self.fires = []
        self.detected_fires = []

        self.refill_radius = 3
        self.fire_detection_radius = 10
        self.fire_fighting_radius = 3
        self.fire_spread_radius = 3
        self.buckets_per_fire = 3
        self.time_steps_before_ash = 60
        self.time_steps_for_new_fire = 10
        self.fire_spread_rate = 0.01


    def valid_points(self,n=1):
        ''' returns a list of lists where there are no obstacles, [[x1,y1],[x2,y2],[x3,y3]....] '''
        # print('n is',n)
        points = []
        for robot in range(n):
            i,j = np.random.randint(self.length),np.random.randint(self.width)
            while self.space[i,j] != 1:
                i,j = np.random.randint(self.length),np.random.randint(self.width)
            points.append([i,j])

        if n==1:
            return points[0]
        else:
            return points
        

    def generate_paths(self):
        self.paths = []
        self.next_steps = []
        for robot_id in range(self.number_of_robots):
            self.assign_goal(robot_id)
            conflicts = [self.robot_positions,self.next_steps]
            # print(conflicts)
            path = a_star.main(self.forest,self.robot_positions[robot_id],self.goal_positions[robot_id], 
                            collisions = [self.robot_positions,self.next_steps])
            if path is None or len(path)<2:        # if path is not found, stay at current location
                path = np.array([self.robot_positions[robot_id],self.robot_positions[robot_id]])
            
            self.paths.append(path)
            try:
                self.next_steps.append([path[1][0],path[1][1]])
            except:
                print('Path:',path)
        # print(self.next_steps)

    def start_fire(self):
        i,j = np.random.randint(self.length),np.random.randint(self.width)
        while self.simulated_space[i,j] != 0:
            i,j = np.random.randint(self.length),np.random.randint(self.width)
        print('! Fire Started at ',[i,j])
        self.simulated_space[i,j] = 10 + self.buckets_per_fire
        self.fires.append([i,j,0])

    
    def spread_fire(self):
        #Go through each of the fires
        for fire in self.fires:
            [x,y] = [fire[0],fire[1]]
            surroundings = self.simulated_space[max(x - self.fire_spread_radius, 0):x + self.fire_spread_radius + 1,
                           max(y - self.fire_spread_radius, 0):y + self.fire_spread_radius + 1]
            space = np.where(surroundings == 0)
            #Get the tress that are next to the fire
            susceptible_trees = [list(point) for point in zip(space[0] + max(x - self.fire_spread_radius, 0),
                                                              space[1] + max(y - self.fire_spread_radius, 0))]
            #Iterate through the trees not on fire
            for tree in susceptible_trees:
                if tree not in self.fires:
                    chance = random.random()
                    #Set the tree on fire if it goes below the rate
                    if chance < self.fire_spread_rate:
                        self.fires.append([tree[0], tree[1], 1])
                        self.simulated_space[tree[0], tree[1]] = 10 + self.buckets_per_fire
                        print('+  Fire Spread to ', [tree[0], tree[1]])


    def detect_fire(self,robot_id):
        [x,y] = self.robot_positions[robot_id]
        observed_area = self.space[max(x-self.fire_detection_radius,0):x+self.fire_detection_radius+1,max(y-self.fire_detection_radius,0):y+self.fire_detection_radius+1]
        space = np.where(observed_area>10)
        fires = [list(point) for point in zip(space[0]+max(x-self.fire_detection_radius,0),
                                              space[1]+max(y-self.fire_detection_radius,0))]
        if len(fires)>0 and type(fires[0])==list:
            for fire in fires:
                if fire not in self.detected_fires:
                    self.detected_fires.append(fire)
                    print('- Fire detected at ',fire)
        elif len(fires)>0 and type(fires)==list and fires not in self.detected_fires:
            self.detected_fires.append(fires)
            print('- Fire detected at, ',fires)


    def extinguish_fire(self,robot_id):
        [x,y] = self.robot_positions[robot_id]
        check_area = self.space[max(x-self.fire_fighting_radius,0):x+self.fire_fighting_radius+1,max(y-self.fire_fighting_radius,0):y+self.fire_fighting_radius+1]
        space = np.where(check_area>10)
        fires = [list(point) for point in zip(space[0]+max(x-self.fire_fighting_radius,0),
                                              space[1]+max(y-self.fire_fighting_radius,0))]
        if len(fires)>0 and self.water[robot_id]:
            self.simulated_space[fires[0][0],fires[0][1]] -= 1
            print('~ Water dropped at',[fires[0][0],fires[0][1]])
            self.water[robot_id] = False
            

    def replenish_water(self,robot_id):
        [x,y] = self.robot_positions[robot_id]
        check_area = self.space[max(x-self.refill_radius,0):x+self.refill_radius+1,max(y-self.refill_radius,0):y+self.refill_radius+1]
        space = np.where(check_area==2)
        reservoir = [list(point) for point in zip(space[0]+max(x-self.refill_radius,0),
                                                  space[1]+max(y-self.refill_radius,0))]
        if len(reservoir) > 0 and self.water[robot_id] == False:
            self.water[robot_id] = True
            self.goal_positions[robot_id] = self.valid_points()
        

    def activity(self):
        
        if self.time_steps%self.time_steps_for_new_fire == 0:
            self.start_fire()
            
        for fire in self.fires:
            if self.simulated_space[fire[0],fire[1]] == 10:
                print('Y Fire Extinguished at ', [fire[0],fire[1]],'!')
                self.simulated_space[fire[0],fire[1]] = 0  # becomes a normal obstacle again
                self.space[fire[0],fire[1]] = 0
                if [fire[0],fire[1]] in self.detected_fires:
                    self.detected_fires.remove([fire[0],fire[1]])
                self.fires.remove(fire)

                if len(self.detected_fires) == 0:
                    print('* Reassinging Goals')
                    self.goal_positions = self.valid_points(self.number_of_robots)
                continue
            
            fire[2] += 1    # increment time that the tree has been burning
            if fire[2] >= self.time_steps_before_ash:
                print('X Burned to ash at ', [fire[0],fire[1]])
                self.simulated_space[fire[0],fire[1]] = 3   # turns to ash
                if [fire[0],fire[1]] in self.detected_fires:
                    self.detected_fires.remove([fire[0],fire[1]])
                self.fires.remove(fire)

        self.spread_fire()

        for robot_id in range(self.number_of_robots):
            # detect fire if nearby
            self.detect_fire(robot_id)

            # extinguish fire
            self.extinguish_fire(robot_id)

            # fill water
            self.replenish_water(robot_id)

            # check for collisions
            if (self.robot_positions[robot_id] in self.robot_positions[:robot_id]) or (self.robot_positions[robot_id] in self.robot_positions[robot_id+1:]):
                print('? Collision')


    def assign_goal(self,robot_id):
        if self.water[robot_id] is not True:
            if euclidian_dist(self.robot_positions[robot_id],self.res1) < euclidian_dist(self.robot_positions[robot_id],self.res2):
                self.goal_positions[robot_id] = self.res1
            else:
                self.goal_positions[robot_id] = self.res2

        elif len(self.detected_fires)>0:
            closest = np.inf
            for fire in self.detected_fires:
                d = euclidian_dist(self.robot_positions[robot_id],fire)
                if d < closest:
                    closest = d
                    temp_goal = fire
            self.goal_positions[robot_id] = temp_goal

        elif euclidian_dist(self.robot_positions[robot_id],self.goal_positions[robot_id])<=3:
            self.goal_positions[robot_id] = self.valid_points()


    def move(self):
        for robot_id in range(self.number_of_robots):
            self.robot_positions[robot_id][0] = self.paths[robot_id][1][0]
            self.robot_positions[robot_id][1] = self.paths[robot_id][1][1]

    
    def step(self):
        self.time_steps += 1
        self.generate_paths()
        self.move()
        self.activity()
        


# FS = Fire_Swarm(4,[50,50],10)
# FS.step()