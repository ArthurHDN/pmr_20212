#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from math import cos, sin, atan2, sqrt
import numpy as np

from DifferentialRobot import *
from RvizMarkerSender import *

def set_simulation_params(P):
    rospy.set_param('/x_goal', P[0])
    rospy.set_param('/y_goal', P[1])
    rospy.set_param('/d', P[2])
    rospy.set_param('/c', P[3])
    rospy.set_param('/p0', P[4])
    rospy.set_param('/eta', P[5])

def get_simulation_params():
    x_goal = float(rospy.get_param('/x_goal'))
    y_goal = float(rospy.get_param('/y_goal'))
    d = float(rospy.get_param('/d'))
    c = float(rospy.get_param('/c'))
    p0 = float(rospy.get_param('/p0'))
    eta = float(rospy.get_param('/eta'))
    return [float(param) for param in [x_goal, y_goal,d,c,p0,eta]]

class ControlNode():
    def __init__(self, freq=10):
        # Init node
        rospy.init_node('control_node')
        # Topics
        self.publisher_robot_path = rospy.Publisher('/robot_path', Path, queue_size=1)
        rospy.Subscriber('/clock', Clock, self.callback_time)
        self.freq = float(freq)
        self.rate = rospy.Rate(self.freq)
        # Robot and curve
        self.robot = DifferentialRobot('/base_pose_ground_truth','/cmd_vel')
        self.target_curve = lambda t, P : np.array( [P[0], P[1], 0.0] )
        # Rviz markers
        self.robot_pose_marker = RvizMarkerSender('/robot_pose_marker','point', color=[1,0.5,0.5,0.5])
        self.robot_cmd_marker = RvizMarkerSender('/robot_cmd_marker','vector', color=[1,0.8,0.8,0.8],id=1)
        self.target_pose_marker = RvizMarkerSender('/target_pose_marker','point', color=[1,0,1,0],id=2)
        # Initialize P,F,path
        self.P = get_simulation_params()
        self.F = (0,0,0)
        self.target = (0,0,0)
        self.path = Path()
        # Pub empty velocity to start
        self.robot.pub_vel(self.F)
        self.rate.sleep()

    def main_loop(self):
        while not rospy.is_shutdown():
            self.F = self.calculate_ref_vel()
            self.robot.pub_vel(self.F)
            self.send_markers()
            self.publish_path()
            self.rate.sleep()

    def calculate_ref_vel(self):
        self.P = get_simulation_params()
        # Attractive
        d = self.P[2]; c = self.P[3]
        q = np.array(self.robot.get_position3D())
        q_goal = self.target_curve(self.time, self.P); self.target = q_goal
        pf = np.linalg.norm(q - q_goal)
        if pf <= d:
            Fatt = -c*(q-q_goal)
        else:
            Fatt = -(d*c/pf)*(q-q_goal)
        # Repulsive
        b = np.array(self.robot.get_closest_obst_position3D())
        p0 = self.P[4]; eta = self.P[5]
        p = np.linalg.norm(q-b)
        if p <= p0:
            Frep = (eta*(1/p - 1/p0)/(p**2))*(q - b)/p
        else:
            Frep = np.array((0,0,0))
        F = Fatt + Frep
        return F

    def send_markers(self):
        # Robot pose
        pose3D=self.robot.get_pose3D()
        self.robot_pose_marker.update_marker(pose3D=pose3D)
        self.robot_pose_marker.pub_marker()
        # Robot cmd
        pose3D=self.robot.get_pose3D()
        pose3D[5] = atan2(self.F[1],self.F[0])
        scale=[sqrt(self.F[0]**2 + self.F[1]**2), 0.2, 0.2]
        self.robot_cmd_marker.update_marker(pose3D=pose3D,scale=scale)
        self.robot_cmd_marker.pub_marker()
        # Target pose
        pose3D=[self.target[0],self.target[1],0,0,0,0]
        self.target_pose_marker.update_marker(pose3D=pose3D)
        self.target_pose_marker.pub_marker()

    def publish_path(self):
        self.path.header.frame_id = '/world'
        self.path.header.stamp = rospy.Time.now()
        pose_msg = PoseStamped()
        pose3D = self.robot.get_pose3D()
        pose_msg.pose.position.x = pose3D[0]
        pose_msg.pose.position.y = pose3D[1]
        pose_msg.pose.position.z = pose3D[2]
        quat = quaternion_from_euler(pose3D[3],pose3D[4],pose3D[5])
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.path.poses.append(pose_msg)
        self.publisher_robot_path.publish(self.path)

    def callback_time(self, data):
        # unit = miliseconds 
        self.time = float(data.clock.secs*1e3 + data.clock.nsecs/1e6)

if __name__ == '__main__':
    try:
        x_goal = input('x_goal = '); y_goal = input('y_goal = ')
        d = 20.; c = 1.0/10; p0 = 10.; eta = 125.
        P = [float(param) for param in [x_goal, y_goal,d,c,p0,eta]]
        set_simulation_params(P)
        control_node = ControlNode()
        control_node.main_loop()
    except rospy.ROSInterruptException:
        pass