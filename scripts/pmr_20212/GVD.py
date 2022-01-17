from PotentialFields import *
from math import atan2, atan, cos, sin, pi, sqrt
from pmr_20212.AuxAlgebra import Inf
import copy
from operator import itemgetter

class GVD():
    _STATE_OBSTACLE_TOO_CLOSE = 1
    _STATE_MEETPOINT_TWO_OBSTACLES = 2
    _STATE_MEETPOINT_THREE_OBSTACLES = 3

    _TOLERANCE = 0.1
    
    def __init__(self, q_goal):
        self.q_goal = q_goal
        self.state = self._STATE_OBSTACLE_TOO_CLOSE
        self.meetpoints = []
        self.min_measurements = []
        #self.aux_min_measurements = []
        self.visited = []
        self.reverse = False
        self.away_wall = True

    def update_params(self,params):
        self.d = params[2]; self.c = params[3]
        self.p0 = params[4]; self.eta = params[5]

    def update_robot(self, q, b, robot_pose2D, robot_center, sensor_max, measurement):
        self.q = q
        self.b = b
        self.robot_pose2D = robot_pose2D
        self.robot_center = robot_center
        self.sensor_max = sensor_max
        self.measurement = measurement

####################################################################

    def sort_sensor_data(self, measurements):
        aux_list = []
        t = len(measurements)
        print(measurements)

        for i in range(0, t):
            if (measurements[i][1] != Inf and measurements[i][1] != -Inf):
                aux_list.append((abs(measurements[i][1]), i/2))

        sorted_sensor_data = sorted(aux_list, key=itemgetter(0))
        #sorted_sensor_data = aux_list.sort()
        #print(sorted_sensor_data)

        return sorted_sensor_data, aux_list



    def move_away_from_obstacle(self, min_angle):
        K = 2.0

        grad_x = cos(min_angle*pi/180 + self.robot_pose2D[2])
        grad_y = sin(min_angle*pi/180 + self.robot_pose2D[2])
        #print(min_angle)
        #print(self.robot_pose2D[2])

        Ux = K * grad_x
        Uy = K * grad_y
        #print(Ux)
        #print(Uy)

        #vx = cos(self.robot_pose2D[2]) * Ux + sin(self.robot_pose2D[2]) * Uy
        #omega = -(sin(self.robot_pose2D[2]) * Ux)/ self.d  + (cos(self.robot_pose2D[2]) * Uy) / self.d 

        F = (Ux, Uy, self.robot_pose2D[2])

        #F = PotentialFields.calculate_repulsive_vector(self.q, self.b, 100.0, 100.0)
        #print(F)
        #print(self.b)

        return F

    def get_min_measurements(self, not_sorted_sensor_data):
        aux_list = []
        t = len(not_sorted_sensor_data)

        for i in range(0, t-1):
            #print(sorted_sensor_data[i])
            previous = not_sorted_sensor_data[i - 1][0]
            next = not_sorted_sensor_data[i + 1][0]
            if(previous > not_sorted_sensor_data[i][0] and  next > not_sorted_sensor_data[i][0]):
                if(not_sorted_sensor_data[i - 1][1] == (not_sorted_sensor_data[i][1] - 1) and not_sorted_sensor_data[i + 1][1] == (not_sorted_sensor_data[i][1] + 1)):
                    aux_list.append(not_sorted_sensor_data[i])
                    #print(aux_list)

        min = copy.deepcopy(aux_list)
        for i in aux_list:
            remove_item = False
            for j in aux_list:
                if((abs(i[1] - j[1]) <= 25) and i != j):
                    if(i[0] < j[0]):
                        remove_item = False
                    else:
                        remove_item = True
            if remove_item:
                min.remove(i)

        #print(min)
        #print(aux_list)

        min.sort()
        return min

    def navigate_in_GVD(self, first_angle, second_angle, reverse):
        K = 2.0

        big = max(first_angle, second_angle)
        low = min(first_angle, second_angle)

        first_angle = big
        second_angle = low
        #print(first_angle)
        #print(second_angle)

        grad_x1 = cos(first_angle*pi/180 + self.robot_pose2D[2])
        grad_y1 = sin(first_angle*pi/180 + self.robot_pose2D[2])

        grad_x2 = cos(second_angle*pi/180 + self.robot_pose2D[2])
        grad_y2 = sin(second_angle*pi/180 + self.robot_pose2D[2])

        grad_x = grad_x1 - grad_x2
        grad_y = grad_y1 - grad_y2

        # Getting the line orthogonal to grad1 - grad2
        aux = grad_x
        grad_x = - grad_y
        grad_y = aux

        Ux = K * grad_x
        Uy = K * grad_y
        #print(Ux)
        #print(Uy)

        #vx = cos(self.robot_pose2D[2]) * Ux + sin(self.robot_pose2D[2]) * Uy
        #omega = -(sin(self.robot_pose2D[2]) * Ux)/ self.d  + (cos(self.robot_pose2D[2]) * Uy) / self.d 

        F = (Ux, Uy, self.robot_pose2D[2])

        if reverse:
            F = (-Ux, -Uy, self.robot_pose2D[2])

        return F

    def obstacle_too_close(self):
        print(self._STATE_OBSTACLE_TOO_CLOSE)
        check_dist = 10

        sensor_data = self.measurement
        #print(sensor_data)
        sorted_sensor_data, not_sorted_sensor_data = self.sort_sensor_data(sensor_data)
        #print(sorted_sensor_data)

        if(sorted_sensor_data != []):
            min_dist_angle = sorted_sensor_data[0][1]
            F = self.move_away_from_obstacle(min_dist_angle)

            self.min_measurements = self.get_min_measurements(not_sorted_sensor_data)
            #print(self.min_measurements)

            if (len(self.min_measurements) > 1):
                check_dist = self.min_measurements[0][0] - self.min_measurements[1][0]
                #print(self.min_measurements[0])
                #print(self.min_measurements[1])

            #print(check_dist)

            if (abs(check_dist) < self._TOLERANCE):
                next_state = self._STATE_MEETPOINT_TWO_OBSTACLES
            else:
                next_state = self._STATE_OBSTACLE_TOO_CLOSE

        else:
            print("No close obstacle!")
            F = (0, 0, 0)
            next_state = self._STATE_OBSTACLE_TOO_CLOSE

        return F, next_state

    def meetpoint_two_obstacles(self):
        print(self._STATE_MEETPOINT_TWO_OBSTACLES)
        sensor_data = self.measurement
        sorted_sensor_data, not_sorted_sensor_data = self.sort_sensor_data(sensor_data)

        self.min_measurements = self.get_min_measurements(not_sorted_sensor_data)

        F = (0.0, 0.0, 0.0)
        next_state = self._STATE_OBSTACLE_TOO_CLOSE

        if (len(self.min_measurements) > 1):
            if (len(self.min_measurements) > 2):
                if(self.min_measurements[0][0] < 0.75 and self.min_measurements[1][0] < 0.75 and self.away_wall):
                        self.reverse = True
                        self.away_wall = False
                    
                if(self.min_measurements[0][0] > 0.75 and self.min_measurements[1][0] > 0.75):
                    self.away_wall = True

                # Check if the robot is at meetpoint
                if (abs(self.min_measurements[0][0] - self.min_measurements[1][0]) < self._TOLERANCE ):
                    if self.away_wall and self.reverse == True:
                        self.reverse = False
                        angle = self.robot_pose2D[2]
                        if angle > 0:
                            angle -= pi
                        else:
                            angle += pi
                            
                        #while not (round(angle, 2) - 0.10 <= round(self.robot_pose2D[2], 2) <= round(angle, 2) + 0.10):
                            #vx = 0.0
                            #omega = 0.5
                            #F = (vx, 0.0, omega)

                    visited_point = False

                    if self.meetpoints != []:
                        for i in range(len(self.meetpoints)):
                            d = sqrt((self.meetpoints[i][0]-self.robot_pose2D[0])**2 + (self.meetpoints[i][1]-self.robot_pose2D[1])**2)
                            if d < 2.0:
                                self.meetpoints[i][2] += 1
                                visited_point = True
                                
                        if not visited_point:
                            self.meetpoints.append([self.robot_pose2D[0], self.robot_pose2D[1], 0])
                            visited_point = False
                    else:
                        self.meetpoints.append([self.robot_pose2D[0], self.robot_pose2D[1], 0])

                    #self.aux_min_measurements = copy.deepcopy(self.min_measurements)
                    #self.aux_min_measurements = sorted(self.aux_min_measurements, key = lambda kv:(kv[1], kv[0])) 
                    next_state = self._STATE_MEETPOINT_THREE_OBSTACLES
                    #print('here1')
                    F = self.navigate_in_GVD(self.min_measurements[0][0], self.min_measurements[1][0], self.reverse)
                    
                else:
                    F = (0.0, 0.0, 0.0)
                    next_state = self._STATE_OBSTACLE_TOO_CLOSE
                    #print('here2')

            else:
                F = self.navigate_in_GVD(self.min_measurements[0][0], self.min_measurements[1][0], self.reverse)
                next_state = self._STATE_OBSTACLE_TOO_CLOSE
                #print('here3')

        return F, next_state

    def meetpoint_three_obstacles(self):
        print(self._STATE_MEETPOINT_THREE_OBSTACLES)
        for i in range(len(self.meetpoints)):
            d = sqrt((self.meetpoints[i][0]-self.robot_pose2D[0])**2 + (self.meetpoints[i][1]-self.robot_pose2D[1])**2)
            if d < 2.0:
                self.visited = self.meetpoints[i][2]

        if (self.visited % 2 == 0):
            F = self.navigate_in_GVD(self.min_measurements[0][0], self.min_measurements[1][0], self.reverse) # moves the robot to the biggest angle obstacles
        else:
            F = self.navigate_in_GVD(self.min_measurements[1][0], self.min_measurements[2][0], self.reverse) # moves the robot to the smallest angle obstacles
        
        next_state = self._STATE_MEETPOINT_TWO_OBSTACLES

        return F, next_state
