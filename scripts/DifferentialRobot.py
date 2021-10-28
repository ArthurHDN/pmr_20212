#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos,sin
from tf.transformations import euler_from_quaternion

from Lidar import *

class DifferentialRobot():
    def __init__(self,pose_topic,cmd_topic):
        # Robot pose 2D
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # For feedback linearization
        self.d = 1.0
        # Ros topics
        self.publisher_cmd_vel = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(pose_topic, Odometry, self.callback_pose)
        # Sensor
        self.range_sensor = Lidar('/base_scan')

    def set_pose2D(self,pose2D):
        self.x = float(pose2D[0])
        self.y = float(pose2D[1])
        self.theta = float(pose2D[2])

    def get_pose2D(self):
        return [self.x, self.y, self.theta]

    def get_pose3D(self):
        return [self.x, self.y, 0.0, 0.0, 0.0, self.theta]

    def get_position3D(self):
        return [self.x, self.y, 0.0]

    def get_closest_obst_position3D(self):
        dist_min = float('inf')
        angle_min = 0
        for angle,dist in self.range_sensor.get_polar_measurement():
            if dist < dist_min:
                dist_min = dist
                angle_min = angle
        # msg = ' = ' + str([self.x, dist_min, self.theta, angle_min])
        # rospy.loginfo(msg)
        if dist_min is float('inf'):
            x_closest = float('inf')
            y_closest = float('inf')
        else:
            x_closest = self.x + dist_min*cos(self.theta + angle_min)
            y_closest = self.y + dist_min*sin(self.theta + angle_min)
        return (x_closest,y_closest,0.0)

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