from platform import node
from PIL import Image
import numpy as np


def display_separate_map(separate_map, n_cells):
    from matplotlib import pyplot as plt
    display_img = np.empty([*separate_map.shape, 3], dtype=np.uint8)
    random_colors = np.random.randint(0, 255, [n_cells+1, 3])
    for cell_id in range(1, n_cells+1):
        display_img[separate_map == cell_id, :] = random_colors[cell_id, :]
    plt.imshow(display_img)
    plt.show()

# ref: https://www.pianshen.com/article/164763958/
# ref: https://www.youtube.com/watch?v=dCNSOVDi3Tc

class Cell():

    def __init__(self, dcmp_img, id, col_skip = 1, border_offset = 0):
        self._dcmp_img = dcmp_img
        self.__id = id
        self.__col_skip = col_skip
        self.__border_offset = border_offset

        self.__waypoints = {}
        self._compute_bous_waypoints()
        self._compute_middle_waypoints()
        
        self.__neighbors = []  
        self._compute_neighbors()

    def get_id(self):
        return self.__id

    def get_waypoints(self, key='bous'):
        return self.__waypoints[key]

    def _compute_middle_waypoints(self):
        cell_pixels = self._dcmp_img == self.__id
        self.__waypoints['middle'] = []
        for j_col in range(int(self.__border_offset/2),cell_pixels.shape[1], self.__col_skip):
            col = cell_pixels[:,j_col]
            if not col.any():
                continue
            i_lins = np.where(col)
            mean = round(0.5*(np.min(i_lins)+np.max(i_lins)))
            self.__waypoints['middle'].append((j_col,mean))

    def _compute_bous_waypoints(self):
        cell_pixels = self._dcmp_img == self.__id
        self.__waypoints['bous'] = []
        top_first = True
        for j_col in range(int(self.__border_offset/2),cell_pixels.shape[1], self.__col_skip):
            col = cell_pixels[:,j_col]
            if not col.any():
                continue
            i_lins = np.where(col)
            if top_first:
                self.__waypoints['bous'].append((j_col,np.min(i_lins)+self.__border_offset))
                self.__waypoints['bous'].append((j_col,np.max(i_lins)-self.__border_offset))
            else:
                self.__waypoints['bous'].append((j_col,np.max(i_lins)-self.__border_offset))
                self.__waypoints['bous'].append((j_col,np.min(i_lins)+self.__border_offset))
            top_first = not top_first

    def _compute_neighbors(self):
        prev_col = self._dcmp_img[:,0]
        for j_col in range(1,self._dcmp_img.shape[1]):
            col = self._dcmp_img[:,j_col]
            neighbors_pixels = []
            if (prev_col == self.__id).any() and not (col == self.__id).any():
                i_lins = np.where(prev_col == self.__id)
                neighbors_pixels = col[i_lins]
            elif not (prev_col == self.__id).any() and (col == self.__id).any():
                i_lins = np.where(col == self.__id)
                neighbors_pixels = prev_col[i_lins]
            for new_neighbor in neighbors_pixels:
                if new_neighbor not in self.__neighbors and new_neighbor != 0:
                    self.__neighbors.append(int(new_neighbor))
            prev_col = col
        self.__neighbors.sort()

    def get_neighbors(self):
        return self.__neighbors


class Boustrophedon():

    def __init__(self, image_path, map_size,q_init,col_skip=1,border_offset=0):
        img = np.array(Image.open(image_path, "r"))
        if len(img.shape) > 2:
            img = img[:,:,0]
        self.__width, self.__height = img.shape
        self.__map_size = map_size
        self.__img = img / np.max(img)
        self.__decomposed_img = []
        self.__n_cells = 0
        self.bcd()
        q_init_pixel = self.get_pixel_by_position2D(q_init)
        node_init = int(max(self.__decomposed_img[q_init_pixel[0],q_init_pixel[1]],1))
        print('node init =', node_init)
        self.plan(node_init=node_init,col_skip=col_skip, border_offset=border_offset)

    def get_n_cells(self):
        return self.__n_cells

    def get_decomposed_img(self):
        return self.__decomposed_img

    def bcd(self):
        current_cell = 1
        current_cells = []
        separate_img = np.copy(self.__img)
        prev_connectivity = 0
        prev_connectivity_parts = []
        for col in range(self.__img.shape[1]):
            connectivity, connective_parts = self._calc_connectivity(self.__img[:, col])
            if prev_connectivity == 0:
                current_cells = []
                for i in range(connectivity):
                    current_cells.append(current_cell)
                    current_cell += 1
            elif connectivity == 0:
                current_cells = []
                continue
            else:
                adj_matrix = self._get_adjacency_matrix(prev_connectivity_parts, connective_parts)
                new_cells = [0] * len(connective_parts)

                for i in range(adj_matrix.shape[0]):
                    if np.sum(adj_matrix[i, :]) == 1:
                        new_cells[np.argwhere(adj_matrix[i, :])[0][0]] = current_cells[i]
                    elif np.sum(adj_matrix[i, :]) > 1:
                        for idx in np.argwhere(adj_matrix[i, :]):
                            new_cells[idx[0]] = current_cell
                            current_cell = current_cell + 1

                for i in range(adj_matrix.shape[1]):
                    if np.sum(adj_matrix[:, i]) > 1:
                        new_cells[i] = current_cell
                        current_cell = current_cell + 1
                    elif np.sum(adj_matrix[:, i]) == 0:
                        new_cells[i] = current_cell
                        current_cell = current_cell + 1
                current_cells = new_cells

            for cell, slice in zip(current_cells, connective_parts):
                separate_img[slice[0]:slice[1], col] = cell
            prev_connectivity = connectivity
            prev_connectivity_parts = connective_parts

        self.__decomposed_img = separate_img
        self.__n_cells = int(np.max(self.__decomposed_img))

    def plan(self,node_init=1,col_skip=1,border_offset=0):
        # Build graph
        self.__cell_array = []
        for i in range(1,self.__n_cells+1):
            self.__cell_array.append(Cell(self.__decomposed_img,i,col_skip=col_skip,border_offset=border_offset))
        self.__path = []
        self._compute_path(node_init)

    def _compute_path(self,node_id):
        if node_id not in self.__path:
            self.__path.append(node_id)
            for neighbour in self.__cell_array[node_id-1].get_neighbors():
                self._compute_path(neighbour)

    def get_transition_waypoints(self,node_id_start,node_id_goal):
        path = self._shortest_path_bfs(node_id_start, node_id_goal)
        # print('From', node_id_start, 'to', node_id_goal, 'path =', path)
        waypoints = []
        if len(path) == 0:
            start_middle_waypoints = self.__cell_array[node_id_start-1].get_waypoints(key='middle')
            waypoints.append( start_middle_waypoints[0] )
            return waypoints
        elif len(path) == 1:
            start_middle_waypoints = self.__cell_array[node_id_start-1].get_waypoints(key='middle')
            goal_middle_waypoints = self.__cell_array[node_id_goal-1].get_waypoints(key='middle')
            if node_id_goal > node_id_start:
                waypoints.append( start_middle_waypoints[-1] )
                waypoints.append( goal_middle_waypoints[0] )
            elif node_id_goal < node_id_start:
                start_middle_waypoints = start_middle_waypoints[::-1]
                goal_middle_waypoints = goal_middle_waypoints[::-1]
                waypoints = waypoints + start_middle_waypoints + goal_middle_waypoints
            return waypoints
        # else:
        # First iteration
        cell_id = node_id_start
        next_cell_id = path[0]
        cell_waypoints = self.__cell_array[cell_id-1].get_waypoints(key='middle')
        # print('start cell',cell_id,'waypoints',cell_waypoints)
        if next_cell_id > cell_id:
            waypoints.append( cell_waypoints[-1] )
        else:
            cell_waypoints = cell_waypoints[::-1]
            waypoints = waypoints + cell_waypoints
        prev_cell_id = cell_id
        # Middle iterations
        for i in range(0,len(path)-1):
            cell_id = path[i]
            next_cell_id = path[i+1]
            cell_waypoints = self.__cell_array[cell_id-1].get_waypoints(key='middle')
            # print('middle cell',cell_id,'waypoints',cell_waypoints)

            if next_cell_id > cell_id and cell_id > prev_cell_id:
                waypoints = waypoints + cell_waypoints
            elif next_cell_id < cell_id and cell_id < prev_cell_id:
                cell_waypoints = cell_waypoints[::-1]
                waypoints = waypoints + cell_waypoints
            elif next_cell_id > cell_id and cell_id < prev_cell_id:
                waypoints.append( cell_waypoints[-1] )
            elif next_cell_id < cell_id and cell_id > prev_cell_id:
                waypoints.append( cell_waypoints[0] )
            else:
                print('ERROR some cell values are equal')

            prev_cell_id = cell_id
        # Last iteration
        cell_id = next_cell_id
        cell_waypoints = self.__cell_array[cell_id-1].get_waypoints(key='middle')
        # print('end cell',cell_id,'waypoints',cell_waypoints)
        if cell_id > prev_cell_id:
            waypoints.append( cell_waypoints[0] )
        else:
            cell_waypoints = cell_waypoints[::-1]
            waypoints = waypoints + cell_waypoints
        return waypoints

    def _shortest_path_bfs(self,node_id_start,node_id_goal):
        visited = [node_id_start]
        queue = [node_id_start]
        pred = [-1 for i in range(self.__n_cells)]
        while queue:
            node_id = queue.pop(0)
            for neighbour in self.__cell_array[node_id-1].get_neighbors():
                if neighbour not in visited:
                    visited.append(neighbour)
                    queue.append(neighbour)
                    pred[neighbour-1] = node_id
                    if neighbour == node_id_goal:
                        break
        path = []
        node_id = node_id_goal
        while pred[node_id-1] != -1:
            path.append(node_id)
            node_id = pred[node_id-1]
        return path[::-1]

    def get_path(self):
        return self.__path

    def get_cell_waypoints(self,cell_id):
        return self.__cell_array[cell_id - 1].get_waypoints()

    def get_pixel_by_position2D(self,q):
        i = (q[0] + self.__map_size[0]/2)*self.__height/self.__map_size[0]
        j = -(q[1] - self.__map_size[1]/2)*self.__height/self.__map_size[1]
        i = min(max(round(i), 0), self.__width-1)
        j = min(max(round(j), 0), self.__height-1)
        return np.array([int(i), int(j)])

    def get_position2D_by_pixel(self, pixel):
        x = (pixel[0]+0.5)*self.__map_size[0]/self.__width - self.__map_size[0]/2
        y = -(pixel[1]+0.5)*self.__map_size[1]/self.__height + self.__map_size[1]/2
        return np.array([x,y])

    def _calc_connectivity(self, slice):
        connectivity = 0
        prev_data = 0
        open_part = False
        connective_parts = []
        for i, data in enumerate(slice):
            if prev_data == 0 and data == 1:
                open_part = True
                start_point = i
            elif prev_data == 1 and data == 0 and open_part:
                open_part = False
                connectivity += 1
                end_point = i
                connective_parts.append((start_point, end_point))
            prev_data = data
        return connectivity, connective_parts

    def _get_adjacency_matrix(self, parts_left, parts_right):
        adjacency_matrix = np.zeros([len(parts_left), len(parts_right)])
        for l, lparts in enumerate(parts_left):
            for r, rparts in enumerate(parts_right):
                if min(lparts[1], rparts[1]) - max(lparts[0], rparts[0]) > 0:
                    adjacency_matrix[l, r] = 1
        return adjacency_matrix


if __name__ == '__main__':
    B = Boustrophedon('../../worlds/map_big_1.png', [205,205], [0,0],col_skip=10)

    dcmp_img = B.get_decomposed_img()
    n_cells = B.get_n_cells()
    print('n_cells = ' + str(n_cells))
    print('path =', B.get_path())
    # print(B.get_transition_waypoints(9,10))
    # print(B.get_transition_waypoints(10,9))
    # print(B.get_transition_waypoints(9,7))
    # print('5 to 3 =', B.get_transition_waypoints(5,3))
    display_separate_map(dcmp_img, n_cells)
