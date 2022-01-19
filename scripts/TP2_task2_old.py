#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import cos, sin, pi, sqrt
from sensor_msgs.msg import LaserScan
import numpy as np
import copy
from operator import itemgetter


class Robot:

    def __init__(self):
        self.d = 0.2
        self.k = 1

        self.robot_pos = [0, 0]
        self.robot_vel = [0, 0]
        self.robot_ori = 0

        self.lidar_data = []
        self.lidar_x = [0] * 360
        self.lidar_y = [0] * 360
        self.l_max = 0
        self.l_min = 0
        self.cmd_vel_msg = Twist()

        self.controlador = Control()

        rospy.init_node("Voronoi", anonymous=True)
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.callback_robot_odom)
        rospy.Subscriber('/base_scan', LaserScan, self.callback_sensor)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


    def callback_robot_odom(self,data):
        self.robot_pos[0] = data.pose.pose.position.x
        self.robot_pos[1] = data.pose.pose.position.y

        x_q = data.pose.pose.orientation.x
        y_q = data.pose.pose.orientation.y
        z_q = data.pose.pose.orientation.z
        w_q = data.pose.pose.orientation.w
        euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

        self.robot_ori = euler[2]


    def callback_sensor(self, data):
        self.lidar_data  = data.ranges
        self.l_max = data.range_max
        self.l_min = data.range_min

        for alfa in range(0, 359):
            sx = (cos(self.robot_ori)*(self.lidar_data[alfa]*cos(np.deg2rad(alfa-180))) - sin(self.robot_ori)*(self.lidar_data[alfa]*sin(np.deg2rad(alfa-180)))) +self.robot_pos[0]
            sy = (sin(self.robot_ori)*(self.lidar_data[alfa]*cos(np.deg2rad(alfa-180))) + cos(self.robot_ori)*(self.lidar_data[alfa]*sin(np.deg2rad(alfa-180)))) +self.robot_pos[1]
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
        K = 2.0
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

        grad_x1 = cos(alfa1*pi/180 + self.robot_ori)
        grad_y1 = sin(alfa1*pi/180 + self.robot_ori)

        grad_x2 = cos(alfa2*pi/180 + self.robot_ori)
        grad_y2 = sin(alfa2*pi/180 + self.robot_ori)

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

        self.cmd_vel_msg.linear.x, self.cmd_vel_msg.angular.z = self.feedback_linearization(Ux,Uy,self.robot_ori)
        self.cmd_vel_msg.linear.x = self.cmd_vel_msg.linear.x/1.5
        if reverse:
            self.cmd_vel_msg.linear.x = -self.cmd_vel_msg.linear.x
        self.pub_cmd_vel.publish(self.cmd_vel_msg)


    def move_away_from_obstacle(self, alfa1):
        K = 2.0
        pos = []

        pos.append(self.lidar_x[alfa1])
        pos.append(self.lidar_y[alfa1])

        alfa = alfa1

        grad_x = cos(alfa*pi/180 + self.robot_ori)
        grad_y = sin(alfa*pi/180 + self.robot_ori)

        Ux = K * grad_x
        Uy = K * grad_y
        #print(Ux)
        #print(Uy)
        #print(self.robot_ori)
        #print(alfa1)
        #print(self.robot_ori)
       
        self.cmd_vel_msg.linear.x, self.cmd_vel_msg.angular.z = self.feedback_linearization(Ux,Uy,self.robot_ori)
        self.pub_cmd_vel.publish(self.cmd_vel_msg)


    def feedback_linearization(self,Ux, Uy, theta_n):
        vx = cos(theta_n) * Ux + sin(theta_n) * Uy
        w = -(sin(theta_n) * Ux)/ self.d  + (cos(theta_n) * Uy) / self.d 

        return vx, w


def explore():
    rospy.sleep(1)

    robot = Robot()
    rate = rospy.Rate(20)
    stage = 0
    reverse = False
    away_wall = True
    meetpoints = []

    while not rospy.is_shutdown():

        if(stage == 0):
            if(robot.lidar_data):
                stage = 1
                s, alfa = robot.sort_sensor_data()
                prev_alfa = alfa[0]

        if (stage == 1):
            s, alfa = robot.sort_sensor_data()

            robot.move_away_from_obstacle(alfa[0])

            local_min = robot.get_min_measurements()
            local_min.sort()

            if (len(local_min) > 1):
                if (abs(local_min[0][0] - local_min[1][0]) < 0.50):
                    stage = 2
        
        if (stage == 2):
            s, alfa = robot.sort_sensor_data()

            local_min = robot.get_min_measurements()
            local_min.sort()
            print("Estado 2")
            
            if (len(local_min) > 1):
                
                if (len(local_min) > 2):
                    if(local_min[0][0] < 0.75 and local_min[1][0] < 0.75 and away_wall):
                        reverse = True
                        away_wall = False
                    
                    if(local_min[0][0] > 0.75 and local_min[1][0] > 0.75):
                        away_wall = True

                    # Check if robot is at meetpoint
                    if (abs(local_min[0][0] - local_min[2][0]) < 0.35 ):
                        if away_wall and reverse == True:
                            reverse = False
                            angle = robot.robot_ori
                            if angle > 0:
                                angle -= pi
                            else:
                                angle += pi

                            while not (round(angle, 2) - 0.10 <= round(robot.robot_ori, 2) <= round(angle, 2) + 0.10):
                                print(round(robot.robot_ori, 2), round(angle, 2))
                                robot.cmd_vel_msg.linear.x = 0.0
                                robot.cmd_vel_msg.angular.z = 0.5
                                robot.pub_cmd_vel.publish(robot.cmd_vel_msg)

                        visited_point = False
                        if meetpoints != []:
                            for i in range(len(meetpoints)):
                                d = robot.compute_distance(meetpoints[i][0], meetpoints[i][1],robot.robot_pos[0], robot.robot_pos[1])
                                if d < 2.0:
                                    meetpoints[i][2] += 1
                                    visited_point = True
                                
                            if not visited_point:
                                meetpoints.append([robot.robot_pos[0], robot.robot_pos[1], 0])
                                visited_point = False
                        else:
                            meetpoints.append([robot.robot_pos[0], robot.robot_pos[1], 0])

                        aux = copy.deepcopy(local_min)
                        aux = sorted(aux, key = lambda kv:(kv[1], kv[0]))
                        stage = 3
                
                robot.navigate_in_GVD(local_min[0][1], local_min[1][1], reverse)
                if (abs(local_min[0][0] - local_min[1][0]) > 0.50):
                    stage = 1
            else:
                robot.navigate_in_GVD(local_min[0][1], reverse)

        if (stage == 3):

            for i in range(len(meetpoints)):
                d = robot.compute_distance(meetpoints[i][0], meetpoints[i][1], robot.robot_pos[0], robot.robot_pos[1])
                if d < 2.0:
                    visited = meetpoints[i][2]

            if (visited % 2 == 0):
                robot.navigate_in_GVD(aux[0][1], aux[1][1], reverse) 
            else:
                robot.navigate_in_GVD(aux[1][1], aux[2][1], reverse) 
            stage = 2

            print("Estado 3")

        rate.sleep()


if __name__ == '__main__':
    try:
        explore()
    except rospy.ROSInterruptException:
        pass