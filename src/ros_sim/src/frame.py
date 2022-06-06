#!/usr/bin/env python

import rospy
import tf_conversions
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
import ros_sim.msg
from nav_msgs.msg import Path
import tf2_ros

pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=3)
pathPub = rospy.Publisher("/turtle1/path", Path, queue_size=3)
class frame():
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    c = -5.54445544
    path = Path()
    def broadcast(self, data):
        pose = PoseStamped()
        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = "world"
        self.t.child_frame_id = "turtle"
        self.t.transform.translation.x = data.x+self.c
        self.t.transform.translation.y = data.y+self.c
        self.t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, data.theta)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
        self.br.sendTransform(self.t)
        self.path.header = self.t.header
        
        pose.header = self.t.header
        pose.pose.position.x = self.t.transform.translation.x
        pose.pose.position.y = self.t.transform.translation.y
        pose.pose.position.z = self.t.transform.translation.z
        pose.pose.orientation = self.t.transform.rotation
        self.path.poses.append(pose)
        pathPub.publish(self.path)

def main():
    rospy.init_node("Frame")
    f = frame()
    rospy.Subscriber("/turtle1/pose", ros_sim.msg.Pose, f.broadcast)
    rospy.spin()

if __name__ == "__main__":
    main()
