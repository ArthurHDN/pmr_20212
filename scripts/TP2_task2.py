#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from math import cos, sin, pi, sqrt
from sensor_msgs.msg import LaserScan
import numpy as np
import copy
from operator import itemgetter


class GVD:
    _STATE_OBSTACLE_TOO_CLOSE = 1
    _STATE_MEETPOINT_TWO_OBSTACLES = 2
    _STATE_MEETPOINT_THREE_OBSTACLES = 3
    _TOLERANCE = 0.5

    def __init__(self):
        self.d = 0.2
        self.k = 0.5

        self.incremental_GVD_pos = [0, 0]
        self.incremental_GVD_vel = [0, 0]
        self.orientation = 0

        self.lidar_data = []
        self.lidar_x = [0] * 360
        self.lidar_y = [0] * 360
        self.l_max = 0
        self.l_min = 0
        self.cmd_vel_msg = Twist()
        self.path = Path()

        rospy.init_node("GVD", anonymous=True)
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.callback_incremental_GVD_odom)
        rospy.Subscriber('/base_scan', LaserScan, self.callback_sensor)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.publisher_robot_path = rospy.Publisher('/robot_path', Path, queue_size=1)

    def callback_incremental_GVD_odom(self,data):
        self.incremental_GVD_pos[0] = data.pose.pose.position.x
        self.incremental_GVD_pos[1] = data.pose.pose.position.y

        x_q = data.pose.pose.orientation.x
        y_q = data.pose.pose.orientation.y
        z_q = data.pose.pose.orientation.z
        w_q = data.pose.pose.orientation.w
        euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

        self.orientation = euler[2]

    def callback_sensor(self, data):
        self.lidar_data  = data.ranges
        self.l_max = data.range_max
        self.l_min = data.range_min

        for alfa in range(0, 359):
            sx = (cos(self.orientation)*(self.lidar_data[alfa]*cos(np.deg2rad(alfa-180))) - sin(self.orientation)*(self.lidar_data[alfa]*sin(np.deg2rad(alfa-180)))) +self.incremental_GVD_pos[0]
            sy = (sin(self.orientation)*(self.lidar_data[alfa]*cos(np.deg2rad(alfa-180))) + cos(self.orientation)*(self.lidar_data[alfa]*sin(np.deg2rad(alfa-180)))) +self.incremental_GVD_pos[1]
            self.lidar_x[alfa]= sx
            self.lidar_y[alfa] = sy 

    def get_min_measurements(self):
        min_measurements = []
        lidar_data = self.lidar_data
        for i in range(1, len(lidar_data)-1):
            if(self.l_min < lidar_data[i] < self.l_max):
                if(lidar_data[i-1] > lidar_data[i] and lidar_data[i+1] > lidar_data[i]):
                    min_measurements.append((lidar_data[i], i))

        # Filtering the results. Sometimes, the laser does not returns the local min correctly
        aux_list = copy.deepcopy(min_measurements)
        for i in min_measurements:
            remove_item = False
            for j in min_measurements:    
                if ((abs(i[1] - j[1]) <= 25) and i != j):
                    if(i[0] < j[0]):
                        remove_item = False
                    else:
                        remove_item = True

            if remove_item:
                aux_list.remove(i)

        aux_list.sort()
        min_measurements.sort()
        # print(aux_list, "|", min_measurements)
        return aux_list

    def compute_distance(self, px1, py1, px2, py2):
        d = sqrt((px1-px2)**2 + (py1-py2)**2)
        return d

    def sort_sensor_data(self):
        s = []
        alfa = []
        cont = 0
        aux_list = []
        
        for i in self.lidar_data:
            a = (i, cont)
            aux_list.append(a)
            cont += 1
        aux_list.sort()
        #print(aux_list)

        for j in aux_list[:40]:
            s.append(j[0])
            alfa.append(j[1])

        return s, alfa
    
    def navigate_in_GVD(self, alfa1, alfa2=0, reverse=False):
        K = 1.0
        pos = []
        maior = max(alfa1, alfa2)
        menor = min(alfa1, alfa2)

        alfa1 = maior
        alfa2 = menor
        #print(alfa1)
        #print(alfa2)
        
        pos.append(self.lidar_x[alfa1])
        pos.append(self.lidar_y[alfa1])

        pos.append(self.lidar_x[alfa2])
        pos.append(self.lidar_y[alfa2])

        grad_x1 = cos(alfa1*pi/180 + self.orientation)
        grad_y1 = sin(alfa1*pi/180 + self.orientation)

        grad_x2 = cos(alfa2*pi/180 + self.orientation)
        grad_y2 = sin(alfa2*pi/180 + self.orientation)

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

        self.cmd_vel_msg.linear.x, self.cmd_vel_msg.angular.z = self.feedback_linearization(Ux,Uy,self.orientation)
        self.cmd_vel_msg.linear.x = self.cmd_vel_msg.linear.x/1.5
        if reverse:
            self.cmd_vel_msg.linear.x = -self.cmd_vel_msg.linear.x
        self.pub_cmd_vel.publish(self.cmd_vel_msg)

    def move_away_from_obstacle(self, alfa1):
        K = 1.0
        pos = []

        pos.append(self.lidar_x[alfa1])
        pos.append(self.lidar_y[alfa1])

        alfa = alfa1

        grad_x = cos(alfa*pi/180 + self.orientation)
        grad_y = sin(alfa*pi/180 + self.orientation)

        Ux = K * grad_x
        Uy = K * grad_y
        #print(Ux)
        #print(Uy)
        #print(self.orientation)
        #print(alfa1)
        #print(self.orientation)
       
        self.cmd_vel_msg.linear.x, self.cmd_vel_msg.angular.z = self.feedback_linearization(Ux,Uy,self.orientation)
        self.pub_cmd_vel.publish(self.cmd_vel_msg)

    def feedback_linearization(self,Ux, Uy, theta_n):
        vx = cos(theta_n) * Ux + sin(theta_n) * Uy
        w = -(sin(theta_n) * Ux)/ self.d  + (cos(theta_n) * Uy) / self.d 

        return vx, w

    def publish_path(self):
        if (self.incremental_GVD_pos[0]**2 + self.incremental_GVD_pos[1]**2) < 1:
            return
        self.path.header.frame_id = '/world'
        self.path.header.stamp = rospy.Time.now()
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.incremental_GVD_pos[0]
        pose_msg.pose.position.y = self.incremental_GVD_pos[1]
        self.path.poses.append(pose_msg)
        self.publisher_robot_path.publish(self.path)


def explore():
    rospy.sleep(1)

    incremental_GVD = GVD()
    rate = rospy.Rate(20)
    state = 0
    reverse = False
    away_wall = True
    meetpoints = []

    while not rospy.is_shutdown():

        # print('state = ' + str(state))

        if(state == 0):
            if(incremental_GVD.lidar_data):
                state = incremental_GVD._STATE_OBSTACLE_TOO_CLOSE
                s, alfa = incremental_GVD.sort_sensor_data()
                prev_alfa = alfa[0]

        if (state == incremental_GVD._STATE_OBSTACLE_TOO_CLOSE):
            s, alfa = incremental_GVD.sort_sensor_data()

            incremental_GVD.move_away_from_obstacle(alfa[0])

            local_min = incremental_GVD.get_min_measurements()
            local_min.sort()
            # print('local_min = ' + str(local_min))
            if (len(local_min) > 1):
                if (abs(local_min[0][0] - local_min[1][0]) < 0.80):
                    state = incremental_GVD._STATE_MEETPOINT_TWO_OBSTACLES
        
        if (state == incremental_GVD._STATE_MEETPOINT_TWO_OBSTACLES):
            s, alfa = incremental_GVD.sort_sensor_data()

            local_min = incremental_GVD.get_min_measurements()
            local_min.sort()
            
            if (len(local_min) > 1):
                
                if (len(local_min) > 2):
                    if(local_min[0][0] < 0.75 and local_min[1][0] < 0.75 and away_wall):
                        reverse = True
                        away_wall = False
                    
                    if(local_min[0][0] > 0.75 and local_min[1][0] > 0.75):
                        away_wall = True

                    # Check if incremental_GVD is at meetpoint
                    if (abs(local_min[0][0] - local_min[2][0]) < 0.35 ):
                        if away_wall and reverse == True:
                            reverse = False
                            angle = incremental_GVD.orientation
                            if angle > 0:
                                angle -= pi
                            else:
                                angle += pi

                            while not (round(angle, 2) - 0.10 <= round(incremental_GVD.orientation, 2) <= round(angle, 2) + 0.10):
                                print(round(incremental_GVD.orientation, 2), round(angle, 2))
                                incremental_GVD.cmd_vel_msg.linear.x = 0.0
                                incremental_GVD.cmd_vel_msg.angular.z = 0.5
                                incremental_GVD.pub_cmd_vel.publish(incremental_GVD.cmd_vel_msg)

                        visited_point = False
                        if meetpoints != []:
                            for i in range(len(meetpoints)):
                                d = incremental_GVD.compute_distance(meetpoints[i][0], meetpoints[i][1],incremental_GVD.incremental_GVD_pos[0], incremental_GVD.incremental_GVD_pos[1])
                                if d < 1.0:
                                    meetpoints[i][2] += 1
                                    visited_point = True
                                
                            if not visited_point:
                                meetpoints.append([incremental_GVD.incremental_GVD_pos[0], incremental_GVD.incremental_GVD_pos[1], 0])
                                visited_point = False
                        else:
                            meetpoints.append([incremental_GVD.incremental_GVD_pos[0], incremental_GVD.incremental_GVD_pos[1], 0])

                        aux = copy.deepcopy(local_min)
                        aux = sorted(aux, key = lambda kv:(kv[1], kv[0]))
                        state = incremental_GVD._STATE_MEETPOINT_THREE_OBSTACLES
                
                incremental_GVD.navigate_in_GVD(local_min[0][1], local_min[1][1], reverse)
                if (abs(local_min[0][0] - local_min[1][0]) > 1.0):
                    state = incremental_GVD._STATE_OBSTACLE_TOO_CLOSE
            else:
                incremental_GVD.navigate_in_GVD(local_min[0][1], reverse)

        if (state == incremental_GVD._STATE_MEETPOINT_THREE_OBSTACLES):

            for i in range(len(meetpoints)):
                d = incremental_GVD.compute_distance(meetpoints[i][0], meetpoints[i][1], incremental_GVD.incremental_GVD_pos[0], incremental_GVD.incremental_GVD_pos[1])
                if d < 1.0:
                    visited = meetpoints[i][2]

            if (visited % 2 == 0):
                incremental_GVD.navigate_in_GVD(aux[0][1], aux[1][1], reverse) 
            else:
                incremental_GVD.navigate_in_GVD(aux[1][1], aux[2][1], reverse) 
            state = incremental_GVD._STATE_MEETPOINT_TWO_OBSTACLES

        incremental_GVD.publish_path()

        rate.sleep()


if __name__ == '__main__':
    try:
        x = raw_input('press enter to start\n')
        explore()
    except rospy.ROSInterruptException:
        pass
