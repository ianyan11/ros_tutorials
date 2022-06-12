#!/usr/bin/env python2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
import cv2

class Vision:
    def __init__(self):
        rospy.Subscriber("/camera/image_raw", Image, self.imageMask)
        rospy.Subscriber("/imu", Imu, self.imuUpdate)

        self.image_publisher = rospy.Publisher("/redImage", Image, queue_size=1)
        self.bridge = CvBridge()        

    def imuUpdate(self, data):
        #type:(..., Imu)->None
        self.linear = data.linear_acceleration
        self.orientation = data.orientation

    def imageMask (self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #red mask filter
        lower_red = np.array([0,0,100])
        upper_red = np.array([100,255,255])
        mask = cv2.inRange(image, lower_red, upper_red)
        #cv2.imshow("mask", mask)
        #cv2.waitKey(1)
        res = cv2.bitwise_and(image,image,mask = mask)
        #add text to image
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(res, "Linear: " + str(self.linear), (0,30), font, 1, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(res, "Orientation: " + str(self.orientation), (0,60), font, 1, (255,255,255), 1, cv2.LINE_AA)

        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))

    


def main():
    rospy.init_node("Vision")
    f = Vision()
    rospy.spin()

if __name__ == "__main__":
    main()