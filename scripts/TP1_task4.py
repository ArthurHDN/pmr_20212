#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from math import atan2, sqrt
import numpy as np

from pmr_20212.DifferentialRobot import *
from pmr_20212.RvizMarkerSender import *
from pmr_20212.WaveFront import *

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
        self.target = lambda t, P : np.array( [P[0], P[1], 0.0] )
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
        rospy.set_param('/x_goal', params[0])
        rospy.set_param('/y_goal', params[1])

    @staticmethod
    def get_simulation_params():
        x_goal = float(rospy.get_param('/x_goal'))
        y_goal = float(rospy.get_param('/y_goal'))
        return [float(param) for param in [x_goal, y_goal]]

    def main_loop(self):
        self.simulation_params = self.get_simulation_params()
        self.target_now = self.target(self.time/1e3, self.simulation_params)
        self.wave_front = WaveFront('/workspaces/arthur_ws/src/pmr_20212/worlds/map_obstacles_3.png', (205, 205))
        self.wave_front.propagate_wave(self.target_now,expand_obstacles=True)
        while not rospy.is_shutdown():
            q = self.robot.get_position2D()
            q_pixel = self.wave_front.get_pixel_by_position2D(q)
            self.F = [10*x for x in self.wave_front.get_next_pixel(q_pixel)]
            self.robot.pub_vel(self.F)
            self.send_markers()
            self.publish_path()
            self.rate.sleep()
            q_cost = self.wave_front.get_pixel_cost(q_pixel)
            if q_cost == self.wave_front._GOAL_COST:
                msg = 'Goal reached!'
                rospy.loginfo(msg)
                break
            if q_cost == self.wave_front._NULL_COST:
                msg = 'No route found to goal.'
                rospy.loginfo(msg)
                break
            if q_cost == self.wave_front._OBSTACLE_COST:
                msg = 'Are you inside a obstacle ??'
                rospy.loginfo(msg)
                break
    
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
        x_goal = input('x_goal = '); y_goal = input('y_goal = ')
        params = [float(param) for param in [x_goal, y_goal]]
        control_node = ControlNode(params)
        control_node.main_loop()
    except rospy.ROSInterruptException:
        pass