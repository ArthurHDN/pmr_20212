from PIL import Image
import sys
sys.setrecursionlimit(1500)

import numpy as np


class AStarNode():

    def __init__(self, q_pixel, parent=None, g=0, h=0):
        self.__parent = parent
        self.__q_pixel = q_pixel
        self.__g = g
        self.__h = h

    def get_parent(self):
        return self.__parent

    def get_pixel(self):
        return self.__q_pixel

    def get_cost_g(self):
        return self.__g

    def get_cost(self):
        return self.__g + self.__h

    def set_g(self,g):
        self.__g = g

    def set_h(self,h):
        self.__h = h
  

class AStar():

    def __init__(self,image_path,map_size):
        self.map_size = [float(x) for x in map_size]
        self.read_image(Image.open(image_path, "r"))

    def read_image(self,image):
        self.image = image
        self.width, self.height = self.image.size
        self.pixels_values = list(self.image.getdata())
        self.scale = [self.map_size[0]/self.width, self.map_size[1]/self.height]

    def find_path(self, q_init, q_goal):
        init_node = AStarNode(self.get_pixel_by_position2D(q_init))
        goal_node = AStarNode(self.get_pixel_by_position2D(q_goal))
        open_set = [init_node]
        closed_set = []
        while len(open_set) > 0:
            current_idx = 0
            current_node = open_set[current_idx]
            for idx,node in enumerate(open_set):
                if node.get_cost() < current_node.get_cost():
                    current_idx = idx
                    current_node = node
            
            open_set.pop(current_idx)
            current_is_in_c = False
            for node in closed_set:
                if (node.get_pixel() == current_node.get_pixel()).all():
                    current_is_in_c = True

            if not current_is_in_c:
                closed_set.append( current_node )

                if (current_node.get_pixel() == goal_node.get_pixel()).all():
                    inverse_path = []
                    while current_node is not None:
                        inverse_path.append(current_node.get_pixel())
                        current_node = current_node.get_parent()
                    path = inverse_path[::-1]
                    return path 

                neighbor_4 = [np.array([1,0]), np.array([0,1]), np.array([-1,0]), np.array([0,-1])]
                neighbor_8 = neighbor_4 + [np.array([1,1]), np.array([-1,1]), np.array([-1,-1]), np.array([1,-1])]
                for next_pixel_increment in neighbor_4:
                    next_pixel = current_node.get_pixel() + next_pixel_increment

                    if self.is_pixel_out_of_limits(next_pixel):
                        continue

                    if not self.is_free_pixel(next_pixel):
                        continue

                    next_is_in_c = False
                    for node in closed_set:
                        if (node.get_pixel() == next_pixel).all():
                            next_is_in_c = True

                    if not next_is_in_c:
                        g = current_node.get_cost_g() + np.linalg.norm(next_pixel_increment)
                        h = np.linalg.norm( next_pixel - goal_node.get_pixel() )
                        open_set.append( AStarNode(next_pixel, parent=current_node, g=g, h=h) )

        return []
                
    def is_pixel_out_of_limits(self,pixel):
        if pixel[0] > self.width-1 or pixel[1] < 0 or pixel[1] > self.height-1 or pixel[1] < 0:
            return True
        else:
            return False

    def is_free_pixel(self,pixel):
        rgb = self.pixels_values[pixel[0] + pixel[1]*self.width]
        return (rgb[0] + rgb[1] + rgb[2])/(3*255) == 1

    def get_pixel_by_position2D(self,q):
        i = min(max(round((q[0] + self.map_size[0]/2)/self.scale[0]), 0), self.width-1)
        j = min(max(round((-q[1] + self.map_size[1]/2)/self.scale[1]), 0), self.height-1)
        return np.array([int(i), int(j)])

    def get_position2D_by_pixel(self, pixel):
        x = self.scale[0]*(pixel[0]+0.5) - self.map_size[0]/2
        y = -self.scale[1]*(pixel[1]+0.5) + self.map_size[1]/2
        return np.array([x,y])

    def get_next_pixel(self,q_pixel):
        pass


