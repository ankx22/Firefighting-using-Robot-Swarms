import math
import numpy as np
import matplotlib.pyplot as plt
import time
import os

# show_animation = True
# show_animation = False

class AStarPlanner:

    def __init__(self, file = 'map30.npy', robot_radius = 1, collisions = None):
        self.nodecount = 0
        self.resolution = 1
        self.rr = robot_radius
        
        if type(file) == str:
            self.map_array = load_map(file)
            self.obstacle_map = self.map_array.astype(np.int32).tolist()
        else: 
            self.map_array = file
            self.obstacle_map = file.astype(np.int32).tolist()
        
        self.collisions = collisions_calc(collisions)
        self.ox,self.oy = obstacles(self.map_array,collisions)

        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y =  self.map_array.shape[0], self.map_array.shape[1]
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        self.motion = self.get_motion_model()
        self.calc_obstacle_map()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)


    def planning(self, start, goal, show_animation = False):
        [sx,sy] = [start[0],start[1]]
        [gx,gy] = [goal[0],goal[1]]
        tic = time.time()
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("A* Path not found")
                return None
                break

            c_id = min(open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            # if current.x == goal_node.x and current.y == goal_node.y:
            #     print("Found goal!")
            #     goal_node.parent_index = current.parent_index
            #     goal_node.cost = current.cost
            #     break
            if ((current.x-goal_node.x)**2+(current.y-goal_node.y)**2)**0.5 <= 2:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        np.append(rx,goal[0]),np.append(ry,goal[1])
        path = np.vstack((rx,ry))
        return path.T

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        rx_s2g = np.flip(rx[1:])
        ry_s2g = np.flip(ry[1:])
        return rx_s2g, ry_s2g

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False
        self.nodecount += 1

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False
        
        if self.collisions != None:
            if [px,py] in self.collisions:
                return False
            
        return True

    def calc_obstacle_map(self):
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                if self.rr <= 1:
                    if self.map_array[ix,iy] == 0:
                        self.obstacle_map[ix][iy] = True
                else:
                    for iox, ioy in zip(self.ox, self.oy):
                        d = math.hypot(iox - x, ioy - y)
                        if d <= self.rr:
                            self.obstacle_map[ix][iy] = True
                            break


    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion


def load_map(file_name):
        map_path = '\\' + file_name
        dir_path = os.path.abspath(os.path.dirname(__file__))
        path = dir_path + map_path
        map_array =  np.load(path)
        return map_array


def collisions_calc(collisions):
    collisions_augmented = []
    if collisions != None:
        for list_1 in collisions:
            if type(list_1) == list:
                for list_2 in list_1:
                    if type(list_2) == list:
                        for element in list_2:
                            if list_2 not in collisions_augmented:
                                collisions_augmented.append(list_2)
                    else:
                        if list_1 not in collisions_augmented:
                            collisions_augmented.append(list_1)
            else:
                collisions_augmented = collisions 
    return collisions_augmented


def obstacles(map_array,collisions = None):
    ox = []
    oy = []
    O = np.argwhere(map_array<1)
    ox = O[:,0].astype(np.int32).tolist()
    oy = O[:,1].astype(np.int32).tolist()
    if collisions != None:
        collisions = collisions_calc(collisions)
        for i in range(len(collisions)):
            ox.append(int(collisions[i][0]))
            oy.append(int(collisions[i][1]))
    return ox,oy


def main(file, start, goal, show_animation = False, show_path = False, collisions = [], robot_radius = 1,node_count = False):
    # print("\nRunning A*:")
    
    a_star = AStarPlanner(file,robot_radius,collisions)

    if show_animation:  # pragma: no cover
        plt.plot(a_star.ox, a_star.oy, ".k")
        plt.plot(start[0], start[1], "om")
        plt.plot(goal[0], goal[1], "xm")
        plt.axis("equal")
    
    path = a_star.planning(start, goal, show_animation)
    
    if path is not None:
        if show_path: 
            plt.title("A*")
            plt.plot(a_star.ox, a_star.oy, ".k")
            plt.plot(start[0], start[1], "om")
            plt.plot(goal[0], goal[1], "xm")
            plt.plot(path[:,0], path[:,1], "-m")
            plt.axis("equal")
            plt.show()
        # print('start:',start,' goal:',goal)
        # print(path)
        if node_count:
            return path, a_star.nodecount
        else:
            return path


if __name__ == '__main__':
    start = [2,0]
    goal = [10,10]
    main('map10.npy', start, goal, show_path = True, show_animation = False, collisions = [[[6,4],[6,3],[6,2],[6,1],[6,0]]])
