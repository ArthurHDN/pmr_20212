#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from math import atan2, sqrt
import numpy as np
import time

from pmr_20212.DifferentialRobot import *
from pmr_20212.RvizMarkerSender import *
from pmr_20212.PotentialFields import PotentialFields
from pmr_20212.Boustrophedon import Boustrophedon,Cell


class ControlNode():

    def __init__(self, freq=5):
        # Init node
        rospy.init_node('control_node')
        self.publisher_robot_path = rospy.Publisher('/robot_path', Path, queue_size=1)
        rospy.Subscriber('/clock', Clock, self.callback_time)
        self.freq = float(freq)
        self.rate = rospy.Rate(self.freq)
        # Robot and goal
        self.robot = DifferentialRobot('/base_pose_ground_truth','/cmd_vel',size=0.18)
        # Rviz markers
        self.robot_pose_marker = RvizMarkerSender('/robot_pose_marker','point', color=[0.2,0.2,1,1])
        self.robot_cmd_marker = RvizMarkerSender('/robot_cmd_marker','vector', color=[1,0.2,0.2,1],id=1)
        # Initialize params,F,path,target
        self.F = (0,0,0)
        self.path = Path()
        # Pub empty velocity to start
        self.robot.pub_vel(self.F)
        self.rate.sleep()

    def main_loop(self):
        time.sleep(2)        
        self.__bous = Boustrophedon('/workspaces/arthur_ws/src/pmr_20212/worlds/map_big_1.png', (70, 70), self.robot.get_position2D(), col_skip=7, border_offset=35)
        path = self.__bous.get_path()

        msg = 'Cells to be visited: ' + str(path)
        rospy.loginfo(msg)

        d = 0.5; c = 5
        p0 = 0.6; eta = 10
        
        while not rospy.is_shutdown():
            prev_cell_id = path[0]
            for cell_id in path:

                msg = 'Next cell: ' + str(cell_id)
                rospy.loginfo(msg)

                waypoints = self.__bous.get_transition_waypoints(prev_cell_id,cell_id) + self.__bous.get_cell_waypoints(cell_id)

                prev_cell_id = cell_id
                
                for q_goal_i_pixel in waypoints:
                    q_goal_i = self.__bous.get_position2D_by_pixel(q_goal_i_pixel)
                    msg = 'Next checkpoint = ' + str(q_goal_i) + ' ,' + str(q_goal_i_pixel) + 'px'
                    rospy.loginfo(msg)
                    D = np.array([100,100])
                    while not np.linalg.norm(D) < 0.25:
                        q = np.array(self.robot.get_position3D())
                        b = np.array(self.robot.get_closest_obst_position3D())
                        q_goal_i = [q_goal_i[0], q_goal_i[1], 0]
                        D = np.array(q) - np.array(q_goal_i)
                        if np.linalg.norm(D) < 0.1:
                            continue
                        self.F = -5*D/np.linalg.norm(D)
                        self.F = PotentialFields.calculate_vector(q, q_goal_i, b, d, c, p0, eta)
                        self.robot.pub_vel(self.F)
                        # self.send_markers()
                        self.publish_path()
                        self.rate.sleep()

            rospy.loginfo('DONE')
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
        x = input('Press enter to start')
        control_node = ControlNode()
        control_node.main_loop()
    except rospy.ROSInterruptException:
        pass
