#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

class Lidar():
    def __init__(self,scan_topic):
        self.angle_min = 0.
        self.angle_max = 0.
        self.angle_increment = 0.
        self.range_min = 0.
        self.range_max = 0.
        self.ranges = []
        self.intensities = []
        
        rospy.Subscriber(scan_topic, LaserScan, self.callback_scan)

    def get_polar_measurement(self):
        angle = self.angle_min
        polar_measurement = []
        for dist in self.ranges:
            angle += self.angle_increment
            if dist == self.range_max:
                polar_measurement.append( (angle,float('inf')) )
            else:
                polar_measurement.append( (angle,dist) )
        return polar_measurement
        
    def callback_scan(self,data):
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.range_min = data.range_min
        self.range_max = data.range_max
        self.ranges = data.ranges
        self.intensities = data.intensities