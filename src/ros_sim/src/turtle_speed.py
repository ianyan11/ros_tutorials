#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=3)
def setSpeed(data):
    msg = Twist()
    msg.linear.x = float(data)
    pub.publish(msg)

def main():
    rospy.init_node("Speed")
    r = rospy.Rate(100)

    while not rospy.is_shutdown():
        setSpeed(sys.argv[1])
        r.sleep()

if __name__ == "__main__":
    main()
