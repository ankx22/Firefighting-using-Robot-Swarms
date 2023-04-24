import numpy as np 
from matplotlib import pyplot as plt

class Obstacle_Field():
    def __init__(self,length,width,desired_density,obstacle_dim = 1,scaling_factor = 1,borders=False):
        self.env_dim_len = length
        self.env_dim_wid = width
        self.desired_density = desired_density
        self.obstacle_dim = obstacle_dim
        self.scaling_factor = scaling_factor    # scaling factor reduces array size to make graph search faster
        self.borders = borders

        self.scaled_field_dim_len = ( self.env_dim_len // self.scaling_factor )
        self.scaled_field_dim_wid = ( self.env_dim_wid // self.scaling_factor )
        self.field_array = np.ones((self.scaled_field_dim_len,self.scaled_field_dim_wid))       # 16x16 numpy array
        self.scaled_obstacle_dim = int(np.ceil(self.obstacle_dim / self.scaling_factor))
        self.current_density = 0
    
    def add_obstacle(self):
        x,y = 5,5
        while np.sum(self.field_array[x:x+2*self.scaled_obstacle_dim,y:y+2*self.scaled_obstacle_dim]) != (2*self.scaled_obstacle_dim)**2 :
            x = np.random.randint(self.scaled_field_dim_len)
            y = np.random.randint(self.scaled_field_dim_wid)
        shape = np.random.randint(5)
        try:
            if shape == 0: # S
                self.field_array[x:x+self.scaled_obstacle_dim*2,y:y+1*self.scaled_obstacle_dim] = 0
                self.field_array[x+1*self.scaled_obstacle_dim:x+3*self.scaled_obstacle_dim,y+1*self.scaled_obstacle_dim:y+2*self.scaled_obstacle_dim] = 0
            elif shape == 1: # T
                self.field_array[x:x+1*self.scaled_obstacle_dim,y:y+3*self.scaled_obstacle_dim] = 0
                self.field_array[x+1*self.scaled_obstacle_dim:x+2*self.scaled_obstacle_dim,y+1*self.scaled_obstacle_dim:y+2*self.scaled_obstacle_dim] = 0
            elif shape == 2: # O
                self.field_array[x:x+1*self.scaled_obstacle_dim,y:y+1*self.scaled_obstacle_dim] = 0
            elif shape == 3: # I
                self.field_array[x:x+3*self.scaled_obstacle_dim,y:y+1*self.scaled_obstacle_dim] = 0
            elif shape == 4: # L
                self.field_array[x:x+3*self.scaled_obstacle_dim,y:y+1*self.scaled_obstacle_dim] = 0
                self.field_array[x:x+1*self.scaled_obstacle_dim,y:y+2*self.scaled_obstacle_dim] = 0
        except:
            print('except')
            pass  

    def density(self):
        empty = np.count_nonzero(self.field_array)
        size =  np.size(self.field_array)
        dens = (size - empty)/size * 100
        return dens
    
    def create_obstacle_field(self):
        while self.current_density < self.desired_density:
            self.add_obstacle()
            self.current_density = self.density()
        if self.borders == True:
            self.field_array[:,0] = 0
            self.field_array[0,:] = 0
            self.field_array[self.scaled_field_dim_len-1,:] = 0
            self.field_array[:,self.scaled_field_dim_wid-1] = 0
        # plt.imshow(self.field_array)
        # print(self.density(self.field_array))

    def start_goal_pos(self):
        start = [np.random.randint(0,self.env_dim_len),np.random.randint(0,self.env_dim_wid)]
        goal = [np.random.randint(0,self.env_dim_len),np.random.randint(0,self.env_dim_wid)]
        while self.field_array[start[0],start[1]] == 0 and self.field_array[goal[0],goal[1]]==0:
            start = [np.random.randint(0,self.env_dim_len),np.random.randint(0,self.env_dim_wid)]
            goal = [np.random.randint(0,self.env_dim_len),np.random.randint(0,self.env_dim_wid)]
        return start, goal

def main(length, width, desired_density, obstacle_dim = 1, scaling_factor = 1, show_obstacle_field = False, borders = False):
    field = Obstacle_Field(length, width, desired_density, obstacle_dim, scaling_factor, borders)
    field.create_obstacle_field()
    if show_obstacle_field:
        plt.imshow(field.field_array)
        plt.show()
    start, goal = field.start_goal_pos()
    return field.field_array, start, goal

    
if __name__ == '__main__':
    length,width = 128,128
    des_den = 20
    main(length,width,des_den,show_obstacle_field=True,borders=True)

