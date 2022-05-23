#!/usr/bin/env python
import cv2
from numpy import dtype
import rospy
import platform
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from matplotlib import pyplot as plt


class Observer():
    def __init__(self):
        if platform.machine() == 'x86_64':
            rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        else:
            rospy.Subscriber("/video_source/raw", Image, self.img_cb)
        

        self.image = np.zeros((0, 0))  # subscriber image
        self.p_img = np.zeros((0, 0))  # processed image
        self.bridge = cv_bridge.CvBridge()  # cv_bridge
        # image publisher
        self.img_pub = rospy.Publisher("filtered_img", Image, queue_size=10)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.detected_color_pub = rospy.Publisher("detected_color", String, queue_size=10)

        frec = 10  # frec var
        r = rospy.Rate(frec)  # Hz
        print("Node initialized {0}Hz".format(frec))
        while not rospy.is_shutdown():
            if self.image.size > 0:
                color = self.detect_line()
                # img to ROS Image msg
                # img_back = self.bridge.cv2_to_imgmsg(self.p_img, encoding="passthrough")
                # self.img_pub.publish(img_back)  # publish image
            r.sleep()

    def detect_line(self, show_img=False):
        image = self.image.copy()
        scale_percent = 20 # percent of original size
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dim = (width, height)
        
        # resize image
        resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
        gray_resized =  cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        cropped = gray_resized[100:,:]
        col_sum = np.sum(cropped, axis=0, dtype="int64")
        # print(np.squeeze(col_sum).shape)
        grad1 = np.gradient(col_sum, axis=0)
        # grad1 = np.minimum(grad1, 0)
        grad2 = np.gradient(grad1, axis=0)
        grad = grad2*grad1
        plt.figure(1)
        plt.plot(grad1)
        plt.show()

        # hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
        
        # # masks
        # maskr = cv2.inRange(hsv, (0, 120, 50), (20, 255, 255))
        # maskg = cv2.inRange(hsv, (49, 39, 130), (75, 255, 255))  # green mask


        # # erode
        # kernel = np.ones((5, 5), np.uint8)
        # erodedr = cv2.erode(maskr, kernel)
        # erodedg = cv2.erode(maskg, kernel)


        # # blob detection
        # # Set up the detector with default parameters.
        # detector = cv2.SimpleBlobDetector_create()
        # keypointsr = detector.detect(erodedr)
        # keypointsg = detector.detect(erodedg)

        # # result
        # self.p_img = cropped
        # blob_sizesr = [blob.size > 1.0 for blob in keypointsr]
        # blob_sizesg = [blob.size > 1.0 for blob in keypointsg]

        # if True in blob_sizesr:
        #     return "RED"
        # elif True in blob_sizesg:
        #     return "GREEN"
        # else:
        #     return None

    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def cleanup(self):
        pass


if __name__ == "__main__":
    rospy.init_node("img_detector", anonymous=True)
    Observer()

