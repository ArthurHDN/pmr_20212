#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from math import cos, sin, atan2, sqrt
import numpy as np

from pmr_20212.DifferentialRobot import *
from pmr_20212.RvizMarkerSender import *

def set_simulation_params(P):
    rospy.set_param('/w', P[0])
    rospy.set_param('/a', P[1])
    rospy.set_param('/k', P[2])

def get_simulation_params():
    w = float(rospy.get_param('/w'))
    a = float(rospy.get_param('/a'))
    k = float(rospy.get_param('/k'))
    return [float(param) for param in [w,a,k]]

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
        # Initialize P,F,path,target
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
        # descobrir qual proximo quadradinho
        dt = 1/self.freq
        self.P = get_simulation_params()
        self.target = self.target_curve(self.time/1e3, self.P)
        position_error = self.target - np.array(self.robot.get_position3D())
        feedfoward = (self.target_curve(self.time/1e3+dt,self.P) - self.target_curve(self.time/1e3-dt,self.P))/(2*dt)
        F = position_error + feedfoward
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
        w = input('x_goal = '); a = input('y_goal = '); k = 10
        P = [float(param) for param in [w,a,k]]
        set_simulation_params(P)
        control_node = ControlNode()
        control_node.main_loop()
    except rospy.ROSInterruptException:
        pass