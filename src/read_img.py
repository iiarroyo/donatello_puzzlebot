#!/usr/bin/env python
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
        # rospy.Subscriber("/video_source/raw", Image, self.img_cb)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_cb)
        # image publisher
        self.img_pub = rospy.Publisher("holahola", Image, queue_size=10)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        frec = 10  # frec var
        r = rospy.Rate(frec)  # Hz
        print("Node initialized {0}Hz".format(frec))
        while not rospy.is_shutdown():
            if self.image.size > 0:
                color = self.detect_color()
                print("Detected color: {0}".format(color))
                cmd = Twist()
                if color == "RED":
                    cmd.linear.x = 0.0
                elif color == "GREEN":
                    cmd.linear.x = 0.3
                else:
                    cmd.linear.x = 0.0
                print(cmd)
                self.cmd_pub.publish(cmd)
                # img to ROS Image msg
                img_back = self.bridge.cv2_to_imgmsg(self.p_img, encoding="passthrough")
                self.img_pub.publish(img_back)  # publish image
            r.sleep()

    def detect_color(self, show_img=False):
        image = self.image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # mask
        lowerr1 = np.array([0, 100, 20])
        uperr1 = np.array([10, 255, 255])

        lowerr2 = np.array([168, 100, 20])
        uperr2 = np.array([179, 255, 255])

        # maskr1 = cv2.inRange(hsv, lowerr1, uperr1)  # red mask
        # maskr2 = cv2.inRange(hsv, lowerr2, uperr2)  # red mask
        # maskr = maskr1 + maskr2
        # maskr = cv2.inRange(hsv, (0, 88, 179), (33, 255, 255))
        maskr = cv2.inRange(hsv, (70, 100, 179), (120, 255, 255))


        # maskg = cv2.inRange(hsv, (49, 39, 130), (98, 255, 255))  # green mask
        maskg = cv2.inRange(hsv, (49, 39, 130), (98, 255, 255))  # green mask

        # apply mask to original image
        targetr = cv2.bitwise_and(image, image, mask=maskr)
        targetg = cv2.bitwise_and(image, image, mask=maskg)


        # threshold
        imgrayr = cv2.cvtColor(targetr, cv2.COLOR_BGR2GRAY)
        imgrayg = cv2.cvtColor(targetg, cv2.COLOR_BGR2GRAY)
        ret, threshr = cv2.threshold(imgrayr, 100, 255, cv2.THRESH_BINARY)
        ret, threshg = cv2.threshold(imgrayg, 100, 255, cv2.THRESH_BINARY)

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
        self.p_img = erodedr
        blob_sizesr = [blob.size > 1.0 for blob in keypointsr]
        blob_sizesg = [blob.size > 1.0 for blob in keypointsg]

        if True in blob_sizesr:
            return "RED"
        elif True in blob_sizesg:
            return "GREEN"
        else:
            return None

    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)

    def cleanup(self):
        pass


if __name__ == "__main__":
    rospy.init_node("img_p", anonymous=True)
    Observer()
