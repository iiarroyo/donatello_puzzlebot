#!/usr/bin/env python 
import cv2, rospy, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
class Observer():
    def __init__(self):
        self.image = np.zeros((0,0))
        self.p_img = np.zeros((0,0))
        self.bridge = cv_bridge.CvBridge()
        # rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_cb)
        self.img_pub = rospy.Publisher("holahola", Image, queue_size=10)
        r = rospy.Rate(10) #Hz 
        while not rospy.is_shutdown():
            if self.image.size > 0:
                self.process_img()
                img_back = self.bridge.cv2_to_imgmsg(self.p_img)  # result image
                self.img_pub.publish(img_back)
            r.sleep()

    def process_img(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # red mask
        maskr = cv2.inRange(hsv, (0, 88, 179), (33, 255,255))
        # green mask
        maskg = cv2.inRange(hsv, (49, 39, 130), (98, 255, 255))
        mask = cv2.bitwise_or(maskg, maskr)
        target = cv2.bitwise_and(self.image, self.image, mask=mask)
        ret, thresh = cv2.threshold(target, 127, 255, cv2.THRESH_BINARY)
        self.p_img = target

    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)


if __name__ == "__main__": 
    rospy.init_node("img_p", anonymous=True) 
    Observer() 