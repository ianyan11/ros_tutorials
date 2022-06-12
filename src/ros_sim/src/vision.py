import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2

class Vision:
    def __init__(self):
        rospy.Subscriber("/camera/image_raw", Image, self.imageMask)
        self.image_publisher = rospy.Publisher("/redImage", Image, queue_size=1)
        

    def imageMask (self, data):
        image = CvBridge.imgmsg_to_cv2(data, "bgr8")
        #red mask filter
        lower_red = np.array([0,100,100])
        upper_red = np.array([10,255,255])
        mask = cv2.inRange(image, lower_red, upper_red)
        #cv2.imshow("mask", mask)
        #cv2.waitKey(1)
        res = cv2.bitwise_and(image,image,mask = mask)
        self.image_publisher.publish(CvBridge.cv2_to_imgmsg(res, "bgr8"))

    


def main():
    rospy.init_node("Vision")
    f = Vision()
    rospy.spin()

if __name__ == "__main__":
    main()