#!/usr/bin/env python 
import cv2
import rospy, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
class Observer():
    def __init__(self):
        self.image = np.zeros((0,0))
        self.bridge = cv_bridge.CvBridge()
        rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        self.img_pub = rospy.Publisher("holahola", Image, queue_size=10)
        r = rospy.Rate(10) #Hz 
        while not rospy.is_shutdown():
            if self.image.size > 0:
                img_back = self.bridge.cv2_to_imgmsg(self.image)  # result image
                self.img_pub.publish(img_back)
            r.sleep()


    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)


if __name__ == "__main__": 
    rospy.init_node("img_p", anonymous=True) 
    Observer() 