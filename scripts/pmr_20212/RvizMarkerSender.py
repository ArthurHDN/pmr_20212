#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler

class RvizMarkerSender():
    def __init__(self, topic, marker_type, color=[1,1,1,1], frame_id='/world', id=0):
        self.publisher = rospy.Publisher(topic, Marker, queue_size=1)
        self.marker = Marker()
        self.marker_type = marker_type
        self.color = color

        self.marker.header.frame_id = frame_id
        self.marker.id = id
        self.marker.action = Marker.ADD
        if self.marker_type == 'point':
            self.marker.type = Marker.SPHERE
        elif self.marker_type == 'vector':
            self.marker.type = Marker.ARROW
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.a = float(self.color[0])
        self.marker.color.r = float(self.color[1])
        self.marker.color.g = float(self.color[2])
        self.marker.color.b = float(self.color[3])

    def update_marker(self, pose3D=[0,0,0,0,0,0],scale=[0.5,0.5,0.5]):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = float(pose3D[0])
        self.marker.pose.position.y = float(pose3D[1])
        self.marker.pose.position.z = float(pose3D[2])
        quaternion = quaternion_from_euler(pose3D[3],pose3D[4],pose3D[5])
        self.marker.pose.orientation.x = quaternion[0]
        self.marker.pose.orientation.y = quaternion[1]
        self.marker.pose.orientation.z = quaternion[2]
        self.marker.pose.orientation.w = quaternion[3]
        self.marker.scale.x = float(scale[0])
        self.marker.scale.y = float(scale[1])
        self.marker.scale.z = float(scale[2])
        # self.marker.color.a = float(color[0])
        # self.marker.color.r = float(color[1])
        # self.marker.color.g = float(color[2])
        # self.marker.color.b = float(color[3])

    def pub_marker(self):
        self.publisher.publish(self.marker)

if __name__ == '__main__':
    try:
        rviz_sender = RvizSender()
    except rospy.ROSInterruptException:
        pass