#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from math import cos, sin, atan2, sqrt
import numpy as np

from pmr_20212.DifferentialRobot import *
from pmr_20212.RvizMarkerSender import *

class ControlNode():
    def __init__(self, params, freq=10):
        # Init node
        self.set_simulation_params(params)
        rospy.init_node('control_node')
        self.publisher_robot_path = rospy.Publisher('/robot_path', Path, queue_size=1)
        rospy.Subscriber('/clock', Clock, self.callback_time)
        self.freq = float(freq)
        self.rate = rospy.Rate(self.freq)
        # Robot and goal
        self.robot = DifferentialRobot('/base_pose_ground_truth','/cmd_vel')
        self.target = lambda t, P : P[1]*cos(P[2]*P[0]*t)*np.array( [cos(P[0]*t), sin(P[0]*t), 0.0] )
        # Rviz markers
        self.robot_pose_marker = RvizMarkerSender('/robot_pose_marker','point', color=[0.2,0.2,1,1])
        self.robot_cmd_marker = RvizMarkerSender('/robot_cmd_marker','vector', color=[1,0.2,0.2,1],id=1)
        self.target_pose_marker = RvizMarkerSender('/target_pose_marker','point', color=[0,1,0,1],id=2)
        # Initialize params,F,path,target
        self.simulation_params = self.get_simulation_params()
        self.F = (0,0,0)
        self.target_now = (0,0,0)
        self.path = Path()
        # Pub empty velocity to start
        self.robot.pub_vel(self.F)
        self.rate.sleep()

    @staticmethod
    def set_simulation_params(params):
        rospy.set_param('/w', params[0])
        rospy.set_param('/a', params[1])
        rospy.set_param('/k', params[2])

    @staticmethod
    def get_simulation_params():
        w = float(rospy.get_param('/w'))
        a = float(rospy.get_param('/a'))
        k = float(rospy.get_param('/k'))
        return [float(param) for param in [w,a,k]]

    def main_loop(self):
        while not rospy.is_shutdown():
            self.F = self.calculate_ref_vel()
            self.robot.pub_vel(self.F)
            self.send_markers()
            self.publish_path()
            self.rate.sleep()

    def calculate_ref_vel(self):
        dt = 1/self.freq
        self.simulation_params = self.get_simulation_params()
        self.target_now = self.target(self.time/1e3, self.simulation_params)
        position_error = self.target_now - np.array(self.robot.get_position3D())
        feedfoward = (self.target(self.time/1e3+dt,self.simulation_params) - self.target(self.time/1e3-dt,self.simulation_params))/(2*dt)
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
        pose3D=[self.target_now[0],self.target_now[1],0,0,0,0]
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
        w = input('w = '); a = input('a = '); k = input('k = ')
        params = [float(param) for param in [w,a,k]]
        control_node = ControlNode(params)
        control_node.main_loop()
    except rospy.ROSInterruptException:
        pass

# curves examples
# https://www.101computing.net/python-turtle-lissajous-curve/
# https://www.101computing.net/python-turtle-spirograph/
