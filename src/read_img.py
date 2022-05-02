#!/usr/bin/env python
import cv2
import rospy
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image


class Observer():
    def __init__(self):
        self.image = np.zeros((0, 0))  # subscriber image
        self.p_img = np.zeros((0, 0))  # processed image
        self.bridge = cv_bridge.CvBridge()  # cv_bridge
        # rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_cb)
        # image publisher
        self.img_pub = rospy.Publisher("holahola", Image, queue_size=10)

        frec = 10  # frec var
        r = rospy.Rate(frec)  # Hz
        print("Node initialized {0}Hz".format(frec))
        while not rospy.is_shutdown():
            if self.image.size > 0:
                self.process_img()  # process img
                # img to ROS Image msg
                img_back = self.bridge.cv2_to_imgmsg(self.p_img)
                self.img_pub.publish(img_back)  # publish image
            r.sleep()

    def process_img(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # mask
        maskr = cv2.inRange(hsv, (0, 88, 179), (33, 255, 255))  # red mask
        maskg = cv2.inRange(hsv, (49, 39, 130), (98, 255, 255))  # green mask

        mask = maskg  # combine masks
        # apply mask to original image
        target = cv2.bitwise_and(self.image, self.image, mask=mask)

        # threshold
        ret, thresh = cv2.threshold(target, 127, 255, cv2.THRESH_BINARY)

        # erode
        kernel = np.ones((5, 5), np.uint8)
        eroded = cv2.erode(thresh, kernel)

        self.p_img = target

    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)


if __name__ == "__main__":
    rospy.init_node("img_p", anonymous=True)
    Observer()

'''
        # mask
        maskr = cv2.inRange(hsv, (0, 88, 179), (33, 255, 255))  # red mask
        maskg = cv2.inRange(hsv, (49, 39, 130), (98, 255, 255))  # green mask

        mask = cv2.bitwise_or(maskg, maskr)  # combine masks
        # apply mask to original image
        target = cv2.bitwise_and(self.image, self.image, mask=mask)

        # threshold
        ret, thresh = cv2.threshold(target, 127, 255, cv2.THRESH_BINARY)

        # erode
        kernel = np.ones((5, 5), np.uint8)
        eroded = cv2.erode(thresh, kernel)

        # blob detection
        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector()
        # keypoints = detector.detect(eroded) # ERROR: segmentation fault

        # result
'''