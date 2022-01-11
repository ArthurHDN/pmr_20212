from PIL import Image
from AuxAlgebra import Inf
import sys
sys.setrecursionlimit(1500)

class WaveFront():
    _GOAL_COST = 2
    _OBSTACLE_COST = Inf
    _NULL_COST = 0
    def __init__(self,image_path,map_size):
        self.map_size = [float(x) for x in map_size]
        self.read_image(Image.open(image_path, "r"))

    def read_image(self,image):
        self.image = image
        self.width, self.height = self.image.size
        self.pixels_values = list(self.image.getdata())
        self.scale = [self.map_size[0]/self.width, self.map_size[1]/self.height]

    def discrete(self,ds):
        dp = [ds[0]/self.scale[0], ds[1]/self.scale[1]]
        self.dp = [max(x,1) for x in dp]
        newsize = [round(x) for x in [self.width/dp[0], self.height/dp[1]]]
        self.read_image(self.image.resize(newsize, Image.ANTIALIAS))

    def get_pixel_by_position2D(self,q):
        x = min(max(round((q[0] + self.map_size[0]/2))/self.scale[0], 0), self.width-1)
        y = min(max(round((-q[1] + self.map_size[1]/2))/self.scale[1], 0), self.height-1)
        return (int(x), int(y))

    def compute_path(self, q_init, q_goal):
        pass

    def propagate_wave(self, q_goal,expand_obstacles=False):
        x_goal_pixel, y_goal_pixel = self.get_pixel_by_position2D(q_goal)
        computed_map = []
        for P in self.pixels_values:
            obstacle = not ((P[0] + P[1] + P[2])/(3*255) > 0) # Black pixel
            if obstacle:
                cost = self._OBSTACLE_COST
            else:
                cost = self._NULL_COST
            computed_map.append( [obstacle,cost] )
        if expand_obstacles:
            self.computed_map = self.expand_obstacles_in_map(computed_map)
        computed_map[y_goal_pixel*self.height + x_goal_pixel][1] = self._GOAL_COST  
        self.computed_map = WaveFront.wave_front(computed_map, (self.width, self.height), self._GOAL_COST, self._NULL_COST)

    def expand_obstacles_in_map(self,computed_map):
        new_computed_map = computed_map
        for i in range(self.height):
            for j in range(self.width):
                if computed_map[i*self.height + j][0] == True:
                    if i+1 < self.height:
                        new_computed_map[(i+1)*self.height + j][1] = self._OBSTACLE_COST
                    if i-1 >= 0:
                        new_computed_map[(i-1)*self.height + j][1] = self._OBSTACLE_COST
                    if j+1 < self.width:
                        new_computed_map[i*self.height + (j+1)][1] = self._OBSTACLE_COST
                    if j-1 >= 0:
                        new_computed_map[i*self.height + (j-1)][1] = self._OBSTACLE_COST
        return new_computed_map

    def get_pixel_cost(self, q_pixel):
        x_pixel, y_pixel = q_pixel
        return self.computed_map[y_pixel*self.height + x_pixel][1]

    def get_next_pixel(self,q_pixel):
        x_pixel, y_pixel = q_pixel
        actual_cost = self.computed_map[y_pixel*self.height + x_pixel][1]
        if actual_cost == self._GOAL_COST:
            # Reached
            return (0,0)
        if y_pixel+1 < self.height:
            if self.computed_map[(y_pixel+1)*self.height + x_pixel][1] == actual_cost - 1:
                return (0,-1)
        if y_pixel-1 >= 0:
            if self.computed_map[(y_pixel-1)*self.height + x_pixel][1] == actual_cost - 1:
                return (0,1)
        if x_pixel+1 < self.width:
            if self.computed_map[y_pixel*self.height + (x_pixel+1)][1] == actual_cost - 1:
                return (1,0)
        if x_pixel-1 >= 0:
            if self.computed_map[y_pixel*self.height + (x_pixel-1)][1] == actual_cost - 1:
                return (-1,0)
        # No route to goal
        return (0,0)
            
    @staticmethod
    def wave_front(computed_map, image_size, actual_cost, null_cost, max_cost=999):
        if actual_cost == max_cost:
            return computed_map
        image_width, image_height = image_size
        next_cost = actual_cost+1
        for i in range(image_height):
            for j in range(image_width):
                P = computed_map[i*image_height + j]
                if P[1] == actual_cost:
                    if i+1 < image_height:
                        if computed_map[(i+1)*image_height + j][1] == null_cost:
                            computed_map[(i+1)*image_height + j][1] = next_cost
                    if i-1 >= 0:
                        if computed_map[(i-1)*image_height + j][1] == null_cost:
                            computed_map[(i-1)*image_height + j][1] = next_cost
                    if j+1 < image_width:
                        if computed_map[i*image_height + (j+1)][1] == null_cost:
                            computed_map[i*image_height + (j+1)][1] = next_cost
                    if j-1 >= 0:
                        if computed_map[i*image_height + (j-1)][1] == null_cost:
                            computed_map[i*image_height + (j-1)][1] = next_cost
        return WaveFront.wave_front(computed_map, image_size, next_cost, null_cost)

    def __str__(self):
        string = ''
        for i in range(self.height):
            for j in range(self.width):
                P = self.computed_map[i*self.height + j]
                if P[1] == self._OBSTACLE_COST:
                    string = string + 'XX' + ' '
                else:
                    string = string + "{:02d}".format(P[1]) + ' '
            string = string + '\n'
        return string

    def show_image(self):
        self.image.show()

    def get_pixels(self):
        return self.pixels
