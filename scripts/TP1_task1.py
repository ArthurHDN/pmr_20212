#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from math import cos, sin, atan2, sqrt, atan, pi
import numpy as np

from pmr_20212.DifferentialRobot import *
from pmr_20212.RvizMarkerSender import *
from pmr_20212.TangentBug import *
from pmr_20212.AuxAlgebra import calc_planar_dist, Inf

class ControlNode():
    _ARRIVE_TOLERANCE = 0.25
    def __init__(self, params, freq=10):
        # Init node
        self.set_simulation_params(params)
        rospy.init_node('control_node')
        self.publisher_robot_path = rospy.Publisher('/robot_path', Path, queue_size=1)
        rospy.Subscriber('/clock', Clock, self.callback_time)
        self.freq = float(freq)
        self.rate = rospy.Rate(self.freq)
        # Robot and curve
        self.robot = DifferentialRobot('/base_pose_ground_truth','/cmd_vel')
        self.target = lambda t, P : np.array( [P[0], P[1], 0.0] )
        # Rviz markers
        self.robot_pose_marker = RvizMarkerSender('/robot_pose_marker','point', color=[0.2,0.2,1,1])
        self.robot_cmd_marker = RvizMarkerSender('/robot_cmd_marker','vector', color=[1,0.2,0.2,1],id=1)
        self.target_pose_marker = RvizMarkerSender('/target_pose_marker','point', color=[0,1,0,1],id=2)# Initialize P,F,path
        self.simulation_params = self.get_simulation_params()
        self.F = (0,0,0)
        self.target_now = (0,0,0)
        self.path = Path()
        self.state = TangentBug._STATE_MOTION_TO_GOAL
        self.d_heuristic_previous = Inf
        self.follow_dir = 1
        self.d_followed = 0.0
        # Pub empty velocity to start
        self.robot.pub_vel(self.F)
        self.rate.sleep()

    @staticmethod
    def set_simulation_params(params):
        rospy.set_param('/x_goal', params[0])
        rospy.set_param('/y_goal', params[1])
        rospy.set_param('/d', params[2])
        rospy.set_param('/c', params[3])
        rospy.set_param('/p0', params[4])
        rospy.set_param('/eta', params[5])

    @staticmethod
    def get_simulation_params():
        x_goal = float(rospy.get_param('/x_goal'))
        y_goal = float(rospy.get_param('/y_goal'))
        d = float(rospy.get_param('/d'))
        c = float(rospy.get_param('/c'))
        p0 = float(rospy.get_param('/p0'))
        eta = float(rospy.get_param('/eta'))
        return [float(param) for param in [x_goal, y_goal,d,c,p0,eta]]

    def main_loop(self):
        # Start in MotionToGoal state
        previous_state = -10
        self.target_now = self.target(self.time, self.simulation_params); 
        q_goal = self.target_now
        q_init_follow_wall = (0,0)
        time_init_follow_wall = self.time

        self.tangent_bug = TangentBug(self.target_now)
        self.tangent_bug.update_params( self.get_simulation_params() )

        next_state = TangentBug._STATE_MOTION_TO_GOAL

        while not rospy.is_shutdown():
            q = np.array(self.robot.get_position3D())
            b = np.array(self.robot.get_closest_obst_position3D())
            continuity_intervals = self.robot.get_continuity_intervals()
            robot_center = self.robot.get_position2D(point='center')
            sensor_max = self.robot.get_sensor_range_max()
            measurement = self.robot.get_measurement_points()

            self.tangent_bug.update_robot(q,b,continuity_intervals,robot_center,sensor_max, measurement)

            if self.state != previous_state:
                previous_state = self.state
                msg = 'State = ' + str(self.state) + ' , t = ' + str(self.time/1000) + ' , q = ' + str(q)
                rospy.loginfo(msg)
            
            if self.state == TangentBug._STATE_MOTION_TO_GOAL:
                F,next_state = self.tangent_bug.motion_to_goal()
                q_init_follow_wall = (0,0)
                time_init_follow_wall = self.time
            elif self.state == TangentBug._STATE_MOTION_TO_OI:
                F,next_state = self.tangent_bug.motion_to_oi()
                q_init_follow_wall = (q[0], q[1])
                time_init_follow_wall = self.time
            elif self.state == TangentBug._STATE_FOLLOW_WALL:
                F,next_state = self.tangent_bug.follow_wall()
            else:
                F = (0.0,0.0,0.0); next_state = TangentBug._STATE_MOTION_TO_GOAL

            self.F = F; self.state = next_state
            self.robot.pub_vel(self.F)
            self.send_markers()
            self.publish_path()
            self.rate.sleep()

            if np.linalg.norm(q - self.target_now) <= self._ARRIVE_TOLERANCE:
                msg = 'Goal reached!'
                rospy.loginfo(msg)
                break
            elif self.state == TangentBug._STATE_FOLLOW_WALL and calc_planar_dist(q_init_follow_wall, q) < self._ARRIVE_TOLERANCE and time_init_follow_wall - self.time > 2e3:
                msg = 'Goal is unreachable!'
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
        d = 2.; c = 1.0; p0 = 1.5; eta = 10.
        params = [float(param) for param in [x_goal, y_goal,d,c,p0,eta]]
        control_node = ControlNode(params)
        control_node.main_loop()
    except rospy.ROSInterruptException:
        pass
