from PotentialFields import *
from math import atan2, atan, cos, sin, pi, sqrt
from pmr_20212.AuxAlgebra import do_2_planar_segments_insersect, calc_planar_dist, Inf, get_planar_orientation_3_points

class TangentBug():
    _STATE_MOTION_TO_GOAL = 1
    _STATE_MOTION_TO_OI = 2
    _STATE_FOLLOW_WALL = 3
    
    def __init__(self,q_goal):
        self.q_goal = q_goal
        self.state = self._STATE_MOTION_TO_GOAL
        self.d_followed = Inf
        self.d_heuristic_previous = Inf
        self.follow_dir = 1

    def update_params(self,params):
        self.d = params[2]; self.c = params[3]
        self.p0 = params[4]; self.eta = params[5]

    def update_robot(self, q, b, continuity_intervals, robot_center, sensor_max, measurement):
        self.q = q
        self.b = b
        self.continuity_intervals = continuity_intervals
        self.robot_center = robot_center
        self.sensor_max = sensor_max
        self.measurement = measurement

    def oi_min_heuristic(self):
        Oi_min = self.continuity_intervals[0][0]
        d_heuristic_min = calc_planar_dist(self.q,Oi_min) + calc_planar_dist(Oi_min,self.q_goal)
        for interval in self.continuity_intervals:
            for Oi in interval:
                d_heuristic = calc_planar_dist(self.q,Oi) + calc_planar_dist(Oi,self.q_goal)
                if d_heuristic_min > d_heuristic:
                    Oi_min = Oi
                    d_heuristic_min = d_heuristic
        Oi_min = np.array([Oi_min[0], Oi_min[1], 0.0])
        return Oi_min, d_heuristic_min

    def is_obstacle_on_way_to_goal(self):
        answer = False
        for interval  in self.continuity_intervals:
            O1,O2 = interval
            obstacle_segment = (O1,O2)    
            robot_goal_angle = atan2( (self.q_goal[1] - self.q[1]), (self.q_goal[0] - self.q[0]) )
            point_behind_robot_range_max = [self.robot_center[0] - self.sensor_max*cos(robot_goal_angle), self.robot_center[1] - self.sensor_max*sin(robot_goal_angle)]
            robot_goal_segment = (point_behind_robot_range_max, self.q_goal)
            if do_2_planar_segments_insersect( robot_goal_segment, obstacle_segment ):
                answer = True
        return answer

    def motion_to_goal(self):
        # Vector
        F = PotentialFields.calculate_vector(self.q, self.q_goal, self.b, self.d, self.c, self.p0, self.eta)
        # Next state transition
        next_state = self._STATE_MOTION_TO_GOAL
        if self.is_obstacle_on_way_to_goal():
            next_state = TangentBug._STATE_MOTION_TO_OI
        # Update control values
        self.d_followed = Inf
        self.d_heuristic_previous = Inf
        # Return
        return F, next_state

    def motion_to_oi(self):
        Oi_min, d_heuristic_min = self.oi_min_heuristic()
        F = PotentialFields.calculate_vector(self.q, Oi_min, self.b, self.d, self.c, self.p0, self.eta)
         # Next state transition
        next_state = self._STATE_MOTION_TO_GOAL
        if self.is_obstacle_on_way_to_goal():
            next_state = TangentBug._STATE_MOTION_TO_OI

        if d_heuristic_min >= self.d_heuristic_previous:
            next_state = TangentBug._STATE_FOLLOW_WALL
        # Update control values
        orientation = get_planar_orientation_3_points(self.q,Oi_min,self.b)
        if orientation == 1:
            self.follow_dir = -1
        else:
            self.follow_dir = 1
        for point in self.measurement:
            d = np.linalg.norm(self.q_goal - np.array([point[0], point[1], 0.0]))
            if self.d_followed > d:
                self.d_followed = d
        self.d_heuristic_previous = d_heuristic_min
        # Return
        return F, next_state

    def follow_wall(self):
        next_state = TangentBug._STATE_FOLLOW_WALL
        if self.b[0] == float('inf'):
            F = (0.0,0.0,0.0)
            next_state = TangentBug._STATE_MOTION_TO_GOAL
        else:
            self.b = np.array([self.b[0],self.b[1],0.0])
            # Vector field to follow wall
            D = self.q - self.b
            E = D - 2*self.p0*D/(np.linalg.norm(D)+1e-6)
            G = -2*atan(np.linalg.norm(E))/pi
            H = self.follow_dir*sqrt(1-G**2+1e-6)
            RD = np.array([-D[1], D[0], 0.0])
            F = self.d*self.c*(G*E/(np.linalg.norm(E)+1e-6) + H*RD/(np.linalg.norm(RD)))
            d_reach = Inf
            # Next state transition
            for interval in self.continuity_intervals:
            # for point in self.robot.get_measurement_points():
                for point in interval:
                    d = np.linalg.norm(self.q_goal - np.array([point[0], point[1], 0.0]))
                    if d_reach > d:
                        d_reach = d
            if d_reach < self.d_followed:
                next_state = TangentBug._STATE_MOTION_TO_GOAL
        return F, next_state

        
        