#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from math import atan2, sqrt
import numpy as np

from pmr_20212.DifferentialRobot import *
from pmr_20212.RvizMarkerSender import *
from pmr_20212.AStar import *


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
        
        self.a_star = AStar('/workspaces/arthur_ws/src/pmr_20212/worlds/map_obstacles_3.png', (205, 205))
        path = self.a_star.find_path(self.robot.get_position2D(),self.target_now)
        msg = 'Path: ' + str(path)
        rospy.loginfo(path)
        while not rospy.is_shutdown():
            if len(path) == 0:
                rospy.loginfo('No route found to goal')
                break
            for q_goal_i_pixel in path:
                q_goal_i = self.a_star.get_position2D_by_pixel(q_goal_i_pixel)
                msg = 'Next checkpoint = ' + str(q_goal_i)
                rospy.loginfo(msg)
                D = np.array([100,100])
                while not np.linalg.norm(D) < 0.1:
                    q = self.robot.get_position2D()
                    D = np.array(q) - np.array(q_goal_i)
                    if np.linalg.norm(D) < 0.1:
                        continue
                    self.F = -2*D/np.linalg.norm(D)
                    self.robot.pub_vel(self.F)
                    self.send_markers()
                    self.publish_path()
                    self.rate.sleep()
            rospy.loginfo('Goal reached')
            break
            
            # if q_cost == self.wave_front._GOAL_COST:
            #     msg = 'Goal reached!'
            #     rospy.loginfo(msg)
            #     break
            # if q_cost == self.wave_front._NULL_COST:
            #     msg = 'No route found to goal.'
            #     rospy.loginfo(msg)
            #     break
            # if q_cost == self.wave_front._OBSTACLE_COST:
            #     msg = 'Are you inside a obstacle ??'
            #     rospy.loginfo(msg)
            #     break
    
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
        # -40, -45 for the center of the spiral
        params = [float(param) for param in [x_goal, y_goal]]
        control_node = ControlNode(params)
        control_node.main_loop()
    except rospy.ROSInterruptException:
        pass
