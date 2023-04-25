import numpy as np
from numpy.linalg import norm
import sys
sys.path.append("utils/")
# sys.path.append("utils/HybridAStar")
import a_star 
# import hybrid_a_star
import obstacle_field_gen
# import parking_lot_gen
# import intersection_gen
from matplotlib import pyplot as plt

def euclidian_dist(p1,p2):
    dist = ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 ) ** 0.5
    return dist

class Voronoi_Cells:
    def __init__(self, N_robots, dimensions, desired_density):
        
        self.length,self.width = dimensions
        self.obstacle_density = desired_density
        self.number_of_robots = N_robots
        self.steps_before_new_cells = 1

        self.space,_,_ = obstacle_field_gen.main(self.length,self.width,self.obstacle_density,borders = True)
        # self.space,self.robot_positions,self.goal_positions = parking_lot_gen.main(self.number_of_robots)
        # self.space,self.robot_positions,self.goal_positions = intersection_gen.main(self.number_of_robots)

        # self.goal_positions = [[15,15,0],[35,35,0],[15,35,0],[35,15,0]]         # hardest test case
        # self.robot_positions = [[30,30,0],[20,20,0],[30,20,0],[20,30,0]]
        
        # self.robot_positions = [[5,5],[45,45],[25,45],[45,25]]         # test case from last week
        # self.goal_positions = [[40,40],[10,10],[45,10],[10,40]]
        
        # self.robot_positions = [[5,5],[45,45],[5,40],[40,5]]         # test case from last week
        # self.goal_positions = [[40,40],[10,10],[45,5],[4,40]]


        self.robot_positions = self.valid_points()                # random test case
        self.goal_positions = self.valid_points()

        # self.space = np.load('map_paper_10.npy')
        # self.robot_positions = [[4, 38], [32, 28], [30, 43], [3, 31]]
        # self.goal_positions = [[25, 6], [3, 3], [8, 27], [43, 19]]
        # print('Starts:',self.robot_positions)
        # print('goals:',self.goal_positions)

        self.segmented_space = self.space
        self.past_positions = []
        self.deadlocked_robots = []
        self.deadlock_threshhold = 10
        self.nodecount = 0

        self.reached = [int(self.robot_positions[robot] == self.goal_positions[robot]) for robot in range(self.number_of_robots) ]
        self.success = 0

        self.vanilla_collisions = 0
        self.collisons_per_step = 0
        for i in range(self.number_of_robots):
            self.vanilla_collisions += obstacles_on_line(self.robot_positions[i],self.goal_positions[i],self.space)
            self.collisons_per_step += obstacles_on_line(self.robot_positions[i],self.goal_positions[i],self.space)/max(1,euclidian_dist(self.robot_positions[i],self.goal_positions[i]))
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

    def voronoi_cells(self):
        self.reached = [int(self.robot_positions[robot] == self.goal_positions[robot]) for robot in range(self.number_of_robots) ]
        for i in range(self.length):
            for j in range(self.width):
                if self.space[i,j] != 0:            # obstacles are call state 0 and empty spaces are cell state 1
                    min,closest = self.width+self.length,None
                    for robot in range(self.number_of_robots):
                        if self.reached[robot] !=1:
                            d = euclidian_dist(self.robot_positions[robot],[i,j])
                            if d < min:
                                min,closest = d,robot
                    if sum(self.reached) == self.number_of_robots:
                        self.success = 1
                        closest = 0
                    self.segmented_space[i,j] = closest+100      # 100,101,102,103...199 will be robot_id to identify closest robot for each cell
        
        for index,position in enumerate(self.robot_positions):
            self.segmented_space[round(position[0]),round(position[1])] = index+200   # demarkate robot positions with cell state 201,202,203,...299 
        for index,position in enumerate(self.goal_positions):
            self.segmented_space[round(position[0]),round(position[1])] = index+300   # demarkate goal positions with cell state 301,302,303,...399 

        return self.segmented_space

    def voronoi_paths(self):
        self.full_paths = []
        self.paths = []
        for robot_id in range(self.number_of_robots):
            if len(self.goal_positions[0]) == 2:
                current_full_path,current_nodecount = a_star.main(self.space,self.robot_positions[robot_id],self.goal_positions[robot_id],
                        collisions = [self.robot_positions,self.goal_positions[:robot_id],self.goal_positions[robot_id+1:]],node_count=True)
                self.yaw_present = False
                self.nodecount += current_nodecount
            elif len(self.goal_positions[0]) == 3:
                try:
                    current_full_path,current_nodecount = hybrid_a_star.main(self.space,self.robot_positions[robot_id],self.goal_positions[robot_id],
                        collisions = [self.robot_positions[:robot_id],self.robot_positions[robot_id+1:]],show_path=False,show_animation=False,node_count=True)
                except:
                    print('No Path')
                    current_full_path = [self.robot_positions[robot_id]]
                self.yaw_present = True
                # self.nodecount += current_nodecount
            # print('\n\n',current_full_path)

            ''' for A* '''
            if not self.yaw_present:
                if len(current_full_path) == 0:
                    print('No Path')
                
                temp_path = []
                if len(current_full_path) == 2:         # for reaching goal when it is adjacent
                    temp_path.append([current_full_path[1][0],current_full_path[1][1]])

                for step in current_full_path:  # checking each element in path
                    if robot_id in self.deadlocked_robots:      # allowed to leave voronoi cell if in deadlock
                        temp_path.append([current_full_path[1][0],current_full_path[1][1]])
                    
                    if self.segmented_space[step[0],step[1]] == robot_id+100:    # if it is in the VC of the robot, go to that point
                        temp_path.append([step[0],step[1]])
                
                if len(temp_path) == 0:     # if no path in VC, stay at current position
                    temp_path.append(self.robot_positions[robot_id])
                
                for count in range(max(self.steps_before_new_cells - len(temp_path),0)):        # if temp_path steps are less than number of steps before next computation, stay in the same spot by repeating last position
                    temp_path.append(temp_path[(len(temp_path)-1)])
                # print('\n',temp_path)
                self.paths.append(temp_path)


            ''' For Hybrid A* '''
            if self.yaw_present:
                if len(current_full_path) == 0:
                    # print('No Path')
                    current_full_path = self.robot_positions[robot_id]
                
                temp_path = []
                # for reaching goal when it is adjacent
                if euclidian_dist([self.robot_positions[robot_id][0],self.robot_positions[robot_id][1]],[self.goal_positions[robot_id][0],self.goal_positions[robot_id][1]]) <= 2 and abs(self.robot_positions[robot_id][2]-self.goal_positions[robot_id][2]) < 10:
                    temp_path.append([self.goal_positions[robot_id][0],self.goal_positions[robot_id][1],self.goal_positions[robot_id][2]])
                    

                for step in current_full_path:  # checking each element in path
                    if robot_id in self.deadlocked_robots:      # allowed to leave voronoi cell if in deadlock
                        temp_path.append([current_full_path[1][0],current_full_path[1][1],current_full_path[1][2]])
                    
                    elif self.segmented_space[min(round(step[0]),self.segmented_space.shape[0]-1),min(round(step[1]),self.segmented_space.shape[0]-1)] == robot_id+100:    # if it is in the VC of the robot, go to that point
                        temp_path.append([step[0],step[1],step[2]])
                
                if len(temp_path) == 0:     # if no path in VC, stay at current position
                    temp_path.append(self.robot_positions[robot_id])
                
                for count in range(max(self.steps_before_new_cells - len(temp_path),0)):        # if temp_path steps are less than number of steps before next computation, stay in the same spot by repeating last position
                    temp_path.append(temp_path[(len(temp_path)-1)])
                # print('\n',temp_path)
                self.paths.append(temp_path)
                # print(self.robot_positions)

    def detect_deadlock(self):
        self.deadlock_detected = 0
        self.deadlocked_robots = []
        self.past_positions.append(self.robot_positions[:])
        
        if len(self.past_positions) > self.deadlock_threshhold:
            self.past_positions.pop(0)

        for robot in range(self.number_of_robots):
            if self.past_positions[0][robot] == self.past_positions[len(self.past_positions)-1][robot] and self.past_positions[0][robot] != self.goal_positions[robot] and len(self.past_positions) == self.deadlock_threshhold:
                self.deadlocked_robots.append(robot)
                self.deadlock_detected = 1

        if self.deadlock_detected:
            print('Deadlock Detected:')


def obstacles_on_line(start,goal,space):

    slope = (goal[1]-start[1])/(max(goal[0]-start[0],0.001))
    constant = goal[1] - slope*goal[0]

    obstacles = np.where(space==0)
    obstacles = np.asarray(obstacles).T
    collisions = 0
    for obstacle in obstacles:
        if obstacle[0]>max(goal[0],start[0]) or obstacle[0]<min(goal[0],start[0]):
            continue
        if obstacle[1]>max(goal[1],start[1]) or obstacle[1]<min(goal[1],start[1]):
            continue
        
        d = (obstacle[1] - slope*obstacle[0] - constant)/np.sqrt(1**2+slope**2)
        
        if abs(d) < 0.8:
            collisions += 1
        # print(d, obstacle)
    # print("Collisions: ",collisions)
    return collisions
                              

def main(n_robots,dimensions,desired_density = 10):
    VC = Voronoi_Cells(n_robots,dimensions,desired_density)
    # VC.voronoi_cells()
    # VC.voronoi_paths()
    # print(VC.obstacle_density)
    return VC.vanilla_collisions

if __name__ == "__main__":
    # main(4,[50,50])
    dims = [50,50]
    n_r = 8
    des_den = np.linspace(0,30,num=31)
    epochs = 100
    print(des_den)
    total = np.zeros(len(des_den))
    for index,d in enumerate(des_den):
        for i in range(epochs):
            total[index] += main(n_r,dims,d)
        total[index] = total[index]/(epochs)
        print(des_den[index],total[index])
    
    plt.plot(des_den,total)
    plt.xlabel('Obstacle Field Density')
    plt.ylabel('Obstacle Collisions')
    plt.show()