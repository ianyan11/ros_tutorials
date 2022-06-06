#!/usr/bin/env python

"""
This program will control a ros turtle x and y position using a Fuzzy controller. 
It's desired position will be defined in the main function.
"""
from math import atan2
import rospy
from geometry_msgs.msg import Twist, PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

class TurtleController():
    def __init__(self):
        self.markerPub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 2)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=3)
        #self.sub = rospy.Subscriber("/turtle1/pose", Pose, lambda _: self.update_error_pose())
        self.desired_point = PointStamped()
        self.error_point = PointStamped()
        self.last_time = rospy.Time.now()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.id = 1
        self.markerArray = MarkerArray()


    def showMarker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = 2
        marker.id = self.id
        self.id += 1
        # Set the scale of the marker
        marker.scale.x = .4
        marker.scale.y = .4
        marker.scale.z = .4
        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # Set the pose of the marker
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        self.markerArray.markers.append(marker)
        self.markerPub.publish(self.markerArray)

    def set_desired_pose(self, x, y):
        # type: (TurtleController, float, float) -> None
        self.desired_point.point.x = x 
        self.desired_point.point.y = y
        self.showMarker(x,y)
        self.desired_point.header.frame_id = "world"
        self.desired_point.header.stamp = rospy.Time.now()
    
    def update_error_pose(self):
        transform = self.tf_buffer.lookup_transform("turtle", "world", rospy.Time(), rospy.Duration(secs=15.0))
        self.error_point = tf2_geometry_msgs.do_transform_point(self.desired_point, transform)
        #print(self.error_point)

    
    def run(self):
        rate = rospy.Rate(20)
        list = rospy.get_param("list")
        list = list + [[0, 0]]
        i = 0
        self.set_desired_pose(list[i][0], list[i][1])
        while not rospy.is_shutdown():
            dt = (rospy.Time.now() - self.last_time).to_sec()
            self.update_error_pose()
            twist = Twist()
            angle = atan2(self.error_point.point.y, self.error_point.point.x)
            if(abs(self.error_point.point.y)>.01 or abs(self.error_point.point.x)>.01):
                if(angle>.007 or angle<-.007):
                    twist.angular.z = np.sign(angle) * 1
                twist.linear.x = np.sign(self.error_point.point.x)*1
            elif(i<len(list)-1):
                i += 1
                self.set_desired_pose(list[i][0], list[i][1])
            else:
                self.pub.publish(Twist())

                break
            self.pub.publish(twist)
            self.last_time = rospy.Time.now()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("TurtleController")
    tc = TurtleController()
    tc.run()
    