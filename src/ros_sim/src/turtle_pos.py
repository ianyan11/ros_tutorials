#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
import tf2_ros
from tf2_geometry_msgs import PointStamped
from visualization_msgs.msg import Marker
import ros_sim.msg

pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=3)
markerPub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
point= []

class turtleControl():
    desPoint = Point()
    p = PointStamped()
    state="start"
    controlState = "rotate"
    def __init__(self):
        self.tf_buf = tf2_ros.Buffer()
        self.listener =tf2_ros.TransformListener(self.tf_buf)

    def showMarker(self, pose, id):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = 2
        marker.id = id
        # Set the scale of the marker
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # Set the pose of the marker
        marker.pose = pose
        markerPub.publish(marker)

    def setUpdatedPose(self, point):
        self.p.point = point
        self.p.header = "world"

    def setDesiredPose(self, _):
        pose_transformed = self.tf_buf.transform("turtle", self.p)
        self.desPoint = pose_transformed.point


    def rotate(self):
        msg = Twist()
        if self.desPoint.x > 0.001:
            msg.angular.z = -.7
        elif self.desPoint.x < -0.001:
            msg.angular.z = .7
        else:
            return True, Twist()
        return False, msg

    def moveForward(self):
        msg = Twist()
        if self.desPoint.y > 0.01:
            msg.linear.x = -1
        elif self.desPoint.y < -0.01:
            msg.linear.x = 1
        else:
            return True, Twist()
        return False, msg

    def control(self):
        if(self.controlState == "rotate"):
            cont, msg =self.control()
            if(cont):
                self.controlState = "forward"
        elif(self.controlState == "forward"):
            cont, msg = self.moveForward()
            if(cont):
                pub.publish(msg)
                return "done"
        pub.publish(msg)

    def stateMachine(self):
        if(self.state =="start"):
            if(len(point)==0):
                self.state=="finish1"
            else:
                self.setDesiredPose(point.pop())
                self.state =="control"
        elif(self.state=="control"):
            if(self.control() == "done"):
                self.state = "start"
        elif(self.state == "finish1"):
            self.setDesiredPose(Point(0,0,0))
            self.state == "finish2"
        elif(self.state == "finish2"):
            if(self.control() == "done"):
                rospy.signal_shutdown("end")

def main():
    rospy.init_node("Pose", anonymous=True)
    r = rospy.Rate(100)
    point.append(Point(0,0,0))
    point.append(Point(6,0,0))
    point.append(Point(1,-4,0))
    point.append(Point(3,2,0))
    point.append(Point(5,-4,0))
    control = turtleControl()
    rospy.Subscriber("/turtle1/pose", ros_sim.msg.Pose, control.setDesiredPose)
    while not rospy.is_shutdown():
        if(control.stateMachine()):
            break
        r.sleep()

if __name__ == "__main__":
    main()
