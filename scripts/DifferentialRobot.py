#!/usr/bin/env python
from geometry_msgs.msg import Twist
from math import cos,sin
from tf.transformations import euler_from_quaternion

class DifferentialRobot():
    def __init__(self):
        # Robot pose 2D
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # For feedback linearization
        self.d = 1.0

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

    def calculate_control_input(self,F):
        # Low level feedback linearization
        vel = Twist()
        vel.linear.x = cos(self.theta)*F[0] + sin(self.theta) * F[1] 
        vel.angular.z = -sin(self.theta)*F[0]/self.d + cos(self.theta)*F[1]/self.d
        return vel

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