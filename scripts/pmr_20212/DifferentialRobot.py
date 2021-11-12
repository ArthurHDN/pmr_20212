#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos,sin,sqrt
from tf.transformations import euler_from_quaternion

from Lidar import *

class DifferentialRobot():
    def __init__(self,pose_topic,cmd_topic,size=0.8):
        # Robot pose 2D
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # For feedback linearization
        self.d = size/2
        # Ros topics
        self.publisher_cmd_vel = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(pose_topic, Odometry, self.callback_pose)
        # Sensor
        self.range_sensor = Lidar('/base_scan')

    def set_pose2D(self,pose2D):
        self.x = float(pose2D[0])
        self.y = float(pose2D[1])
        self.theta = float(pose2D[2])

    def get_position2D(self, point = 'controlled'):
        if point == 'controlled':
            return [self.x + self.d*cos(self.theta), self.y + self.d*cos(self.theta), 0.0]
        if point == 'center':
            return [self.x, self.y]

    def get_position3D(self, point = 'controlled'):
        position2D = self.get_position2D(point = point)
        return [position2D[0], position2D[1], 0.0]
    
    def get_pose2D(self, point = 'controlled'):
        position2D = self.get_position2D(point = point)
        return [position2D[0], position2D[1], self.theta]

    def get_pose3D(self, point = 'controlled'):
        pose2D = self.get_pose2D(point = point)
        return [pose2D[0], pose2D[1], 0.0, 0.0, 0.0, pose2D[2]]

    def get_closest_obst_position3D(self):
        dist_min = float('inf')
        angle_min = 0
        for angle,dist in self.range_sensor.get_polar_measurement():
            if dist < dist_min:
                dist_min = dist
                angle_min = angle
        if dist_min is float('inf'):
            x_closest = float('inf')
            y_closest = float('inf')
        else:
            x_closest = self.x + dist_min*cos(self.theta + angle_min)
            y_closest = self.y + dist_min*sin(self.theta + angle_min)
        return (x_closest,y_closest,0.0)

    def get_measurement_points(self):
        measurement_points = []
        for angle,dist in self.range_sensor.get_polar_measurement():
            x = self.x + dist*cos(self.theta + angle)
            y = self.y + dist*sin(self.theta + angle)
            measurement_points.append( (x,y) )
        return measurement_points

    def get_continuity_intervals(self):
        measurement = self.range_sensor.get_polar_measurement()
        polar_continuity_intervals = self.range_sensor.calc_measurement_continuity_intervals(measurement)
        continuity_intervals = []
        for polar_interval in polar_continuity_intervals:
            interval = []
            for angle,dist in polar_interval:
                x = self.x + dist*cos(self.theta + angle)
                y = self.y + dist*sin(self.theta + angle)
                interval.append( (x,y) )
            continuity_intervals.append( interval )
        return continuity_intervals

    def calculate_control_input(self,F):
        # Low level feedback linearization
        vel = Twist()
        vel.linear.x = cos(self.theta)*F[0] + sin(self.theta) * F[1] 
        vel.angular.z = -sin(self.theta)*F[0]/self.d + cos(self.theta)*F[1]/self.d
        return vel

    def pub_vel(self,F):
        u = self.calculate_control_input(F)
        self.publisher_cmd_vel.publish(u)

    def callback_pose(self,data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        eul = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        theta = eul[2]
        pose2D = [x,y,theta]
        self.set_pose2D(pose2D)

# Unit test
if __name__ == '__main__':
    # N/A
	pass