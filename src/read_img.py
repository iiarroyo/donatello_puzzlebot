#!/usr/bin/env pythont
import cv2
import rospy
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Observer():
    def __init__(self):
        self.image = np.zeros((0, 0))  # subscriber image
        self.p_img = np.zeros((0, 0))  # processed image
        self.bridge = cv_bridge.CvBridge()  # cv_bridge
        # rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_cb)
        # image publisher
        self.img_pub = rospy.Publisher("holahola", Image, queue_size=10)
        self.cmd_pub = rospy.Publisher("cmd_vel", Image, queue_size=10)

        frec = 10  # frec var
        r = rospy.Rate(frec)  # Hz
        print("Node initialized {0}Hz".format(frec))
        while not rospy.is_shutdown():
            if self.image.size > 0:
                detected_color = self.detect_color()  # process img
                color = detected_color()
                print("Detected color: {0}".format(color))
                cmd = Twist()
                if color == "RED":
                    cmd.linear.x = 0.0
                elif color == "GREEN":
                    cmd.linear.x = 0.1
                else:
                    cmd.linear.x = 0.0
                print(cmd)
                self.cmd_pub.publish(cmd)
                # img to ROS Image msg
                img_back = self.bridge.cv2_to_imgmsg(self.p_img, encoding="passthrough")
                self.img_pub.publish(img_back)  # publish image
            r.sleep()

    def detect_color(self, show_img=False):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # mask
        maskr = cv2.inRange(hsv, (0, 88, 179), (33, 255, 255))  # red mask
        maskg = cv2.inRange(hsv, (49, 39, 130), (98, 255, 255))  # green mask
        # apply mask to original image
        targetr = cv2.bitwise_and(self.image, self.image, mask=maskr)
        targetg = cv2.bitwise_and(self.image, self.image, mask=maskg)


        # threshold
        imgrayr = cv2.cvtColor(targetr, cv2.COLOR_BGR2GRAY)
        imgrayg = cv2.cvtColor(targetg, cv2.COLOR_BGR2GRAY)
        ret, threshr = cv2.threshold(imgrayr, 200, 255, cv2.THRESH_BINARY)
        ret, threshg = cv2.threshold(imgrayg, 200, 255, cv2.THRESH_BINARY)

        # erode
        kernel = np.ones((5, 5), np.uint8)
        erodedr = cv2.erode(threshr, kernel)
        erodedg = cv2.erode(threshg, kernel)


        # blob detection
        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create()
        keypointsr = detector.detect(erodedr)
        keypointsg = detector.detect(erodedg)

        # result
        self.p_img = erodedg
        blob_sizesr = [blob.size > 1.0 for blob in keypointsr]
        blob_sizesg = [blob.size > 1.0 for blob in keypointsg]

        if True in blob_sizesr:
            return "RED"
        elif True in blob_sizesg:
            return "GREEN"

    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def cleanup(self):
        pass


if __name__ == "__main__":
    rospy.init_node("img_p", anonymous=True)
    Observer()
