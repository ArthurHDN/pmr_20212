#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

from AuxAlgebra import Inf

class Lidar():
    def __init__(self,scan_topic):
        self.angle_min = 0.
        self.angle_max = 0.
        self.angle_increment = 0.
        self.range_min = 0.
        self.range_max = 0.
        self.ranges = []
        self.intensities = []
        self._SENSOR_INF = Inf
        
        rospy.Subscriber(scan_topic, LaserScan, self.callback_scan)

    def get_polar_measurement(self):
        angle = self.angle_min
        polar_measurement = []
        for dist in self.ranges:
            if dist == self.range_max:
                polar_measurement.append( (angle,self._SENSOR_INF) )
            else:
                polar_measurement.append( (angle,dist) )
            angle += self.angle_increment
        return polar_measurement
    
    def calc_measurement_continuity_intervals(self,measurement):
        continuity_invervals = []
        for i in range(1,len(measurement)-1):
            if measurement[i][1] != self._SENSOR_INF:
                if measurement[i-1][1] == self._SENSOR_INF:
                    continuity_inverval_start = measurement[i]
                elif i == 1:
                    continuity_inverval_start = measurement[i-1]
                if measurement[i+1][1] == self._SENSOR_INF:
                    continuity_inverval_end = measurement[i]
                    continuity_invervals.append((continuity_inverval_start,continuity_inverval_end))
                elif i == len(measurement)-2:
                    continuity_inverval_end = measurement[i+1]
                    continuity_invervals.append((continuity_inverval_start,continuity_inverval_end))
        return continuity_invervals
        
    def callback_scan(self,data):
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.range_min = data.range_min
        self.range_max = data.range_max
        self.ranges = data.ranges
        self.intensities = data.intensities