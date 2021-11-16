#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from math import cos, sin, atan2, sqrt, atan, pi
import numpy as np

from pmr_20212.DifferentialRobot import *
from pmr_20212.RvizMarkerSender import *
from pmr_20212.AuxAlgebra import do_2_planar_segments_insersect, calc_planar_dist, Inf, get_planar_orientation_3_points

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

class TangentBugStates():
    MotionToGoal = 1
    FollowOi = 2
    FollowWall = 3

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
        self.state = TangentBugStates.MotionToGoal
        self.d_heuristic_previous = Inf
        self.follow_dir = 1
        self.d_followed = 0.0
        # Pub empty velocity to start
        self.robot.pub_vel(self.F)
        self.rate.sleep()

    def main_loop(self):
        # Start in MotionToGoal state
        self.state = TangentBugStates.MotionToGoal
        while not rospy.is_shutdown():
            self.P = get_simulation_params()
            d = self.P[2]; c = self.P[3]
            p0 = self.P[4]; eta = self.P[5]
            q_goal = self.target_curve(self.time, self.P); self.target = q_goal
            q = np.array(self.robot.get_position3D())
            continuity_invervals = self.robot.get_continuity_intervals()

            msg = 'State = ' + str(self.state)
            rospy.loginfo(msg)

            # MotionToGoal
            if self.state == TangentBugStates.MotionToGoal:
                # Potential Field to goal
                pf = np.linalg.norm(q - q_goal)
                if pf <= d:
                    Fatt = -c*(q-q_goal)
                else:
                    Fatt = -(d*c/pf)*(q-q_goal)
                b = np.array(self.robot.get_closest_obst_position3D())
                p = np.linalg.norm(q-b)
                if p <= p0:
                    Frep = (eta*(1/p - 1/p0)/(p**2))*(q - b)/p
                else:
                    Frep = np.array((0,0,0))
                F = Fatt + Frep
                self.d_followed = Inf
                # Next state transition
                for interval  in continuity_invervals:
                    O1,O2 = interval
                    obstacle_segment = (O1,O2)

                    robot_center = self.robot.get_position2D(point='center')
                    robot_goal_angle = atan2( (q_goal[1] - q[1]), (q_goal[0] - q[0]) )
                    point_behind_robot_range_max = [robot_center[0] - self.robot.get_sensor_range_max()*cos(robot_goal_angle), robot_center[1] - self.robot.get_sensor_range_max()*sin(robot_goal_angle)]
                    robot_goal_segment = (point_behind_robot_range_max, q_goal)

                    if do_2_planar_segments_insersect( robot_goal_segment, obstacle_segment ):
                        self.state = TangentBugStates.FollowOi
                        self.d_heuristic_previous = Inf
            # FollowOi 
            elif self.state == TangentBugStates.FollowOi:
                Oi_min = continuity_invervals[0][0]
                d_heuristic_min = calc_planar_dist(q,Oi_min) + calc_planar_dist(Oi_min,q_goal)
                for interval in continuity_invervals:
                    for Oi in interval:
                        d_heuristic = calc_planar_dist(q,Oi) + calc_planar_dist(Oi,q_goal)
                        if d_heuristic_min > d_heuristic:
                            Oi_min = Oi
                            d_heuristic_min = d_heuristic
                Oi_min = np.array([Oi_min[0], Oi_min[1], 0.0])
                #### Oi_min = Oi_min + p0*(q - Oi_min)/np.linalg.norm(q - Oi_min)
                # Potential Field To Oi that minimizes d_heuristic
                pf = np.linalg.norm(q - Oi_min)
                if pf <= d/10:
                    Fatt = (0.0,0.0,0.0)
                else:
                    Fatt = -(d*c/pf)*(q-Oi_min)
                b = np.array(self.robot.get_closest_obst_position3D())
                p = np.linalg.norm(q-b)
                if p <= p0:
                    Frep = (eta*(1/p - 1/p0)/(p**2))*(q - b)/p
                else:
                    Frep = np.array((0,0,0))
                F = Fatt + Frep
                # Next state transition
                self.state = TangentBugStates.MotionToGoal
                for interval  in continuity_invervals:
                    O1,O2 = interval
                    if do_2_planar_segments_insersect( (q,q_goal), (O1,O2) ):
                        self.state = TangentBugStates.FollowOi
                if d_heuristic_min >= self.d_heuristic_previous:
                    self.state = TangentBugStates.FollowWall
                    self.d_followed = Inf
                    orientation = get_planar_orientation_3_points(q,Oi_min,self.robot.get_closest_obst_position3D())
                    if orientation == 1:
                        self.follow_dir = -1
                    else:
                        self.follow_dir = 1
                    for point in self.robot.get_measurement_points():
                        d = np.linalg.norm(q_goal - np.array([point[0], point[1], 0.0]))
                        if self.d_followed > d:
                            self.d_followed = d
                # Update d_heuristic
                self.d_heuristic_previous = d_heuristic_min
            # FollowWall
            elif self.state == TangentBugStates.FollowWall:
                closest_obst = self.robot.get_closest_obst_position3D()
                if closest_obst[0] == float('inf'):
                    F = (0.0,0.0,0.0)
                    self.state = TangentBugStates.MotionToGoal
                else:
                    closest_obst = np.array([closest_obst[0],closest_obst[1],0.0])
                    # Vector field to follow wall
                    D = q - closest_obst
                    E = D - 2*p0*D/(np.linalg.norm(D)+1e-6)
                    G = -2*atan(np.linalg.norm(E))/pi
                    H = self.follow_dir*sqrt(1-G**2+1e-6)
                    RD = np.array([-D[1], D[0], 0.0])
                    F = d*c*(G*E/(np.linalg.norm(E)+1e-6) + H*RD/(np.linalg.norm(RD)))
                    d_reach = Inf
                    # Next state transition
                    for interval in continuity_invervals:
                    # for point in self.robot.get_measurement_points():
                        for point in interval:
                            d = np.linalg.norm(q_goal - np.array([point[0], point[1], 0.0]))
                            if d_reach > d:
                                d_reach = d
                    if d_reach < self.d_followed:
                        self.state = TangentBugStates.MotionToGoal
                    # self.d_followed = d_reach
            # No state
            else:
                F = (0.0,0.0,0.0)
                self.state = TangentBugStates.MotionToGoal

            self.F = F
            self.robot.pub_vel(self.F)
            self.send_markers()
            self.publish_path()
            self.rate.sleep()

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
        d = 2.; c = 1.0; p0 = 1.5; eta = 10.
        P = [float(param) for param in [x_goal, y_goal,d,c,p0,eta]]
        set_simulation_params(P)
        control_node = ControlNode()
        control_node.main_loop()
    except rospy.ROSInterruptException:
        pass