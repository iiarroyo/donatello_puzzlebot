#!/usr/bin/env python
import cv2
import rospy
import platform
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def empty(x):
    pass

# from the side
redh_lower_side = 133
reds_lower_side = 41
redv_lower_side = 147
redh_upper_side = 259
reds_upper_side = 255
redv_upper_side = 255

# upfront
redh_lower_front = 0
reds_lower_front = 72
redv_lower_front = 43
redh_upper_front = 33
reds_upper_front = 255
redv_upper_front = 255

greenh_lower = 63
greens_lower = 69
greenv_lower = 100
greenh_upper = 93
greens_upper = 255
greenv_upper = 255


# cv2.namedWindow("Parameters", cv2.WINDOW_NORMAL)
# # cv2.resizeWindow("Parameters",640,240)

# cv2.createTrackbar("redh_lower", "Parameters", 0, 500, empty)
# cv2.createTrackbar("reds_lower", "Parameters", 0, 500, empty)
# cv2.createTrackbar("redv_lower", "Parameters", 50, 500, empty)
# cv2.createTrackbar("redh_upper", "Parameters", 20, 500, empty)
# cv2.createTrackbar("reds_upper", "Parameters", 255, 500, empty)
# cv2.createTrackbar("redv_upper", "Parameters", 255, 500, empty)

# cv2.createTrackbar("greenh_lower", "Parameters", 0, 500, empty)
# cv2.createTrackbar("greens_lower", "Parameters", 0, 500, empty)
# cv2.createTrackbar("greenv_lower", "Parameters", 50, 500, empty)
# cv2.createTrackbar("greenh_upper", "Parameters", 20, 500, empty)
# cv2.createTrackbar("greens_upper", "Parameters", 255, 500, empty)
# cv2.createTrackbar("greenv_upper", "Parameters", 255, 500, empty)

# cv2.setTrackbarPos("redh_lower", "Parameters", redh_lower_side)
# cv2.setTrackbarPos("reds_lower", "Parameters", reds_lower_side)
# cv2.setTrackbarPos("redv_lower", "Parameters", redv_lower_side)
# cv2.setTrackbarPos("redh_upper", "Parameters", redh_upper_side)
# cv2.setTrackbarPos("reds_upper", "Parameters", reds_upper_side)
# cv2.setTrackbarPos("redv_upper", "Parameters", redv_upper_side)

# cv2.setTrackbarPos("greenh_lower", "Parameters", greenh_lower)
# cv2.setTrackbarPos("greens_lower", "Parameters", greens_lower)
# cv2.setTrackbarPos("greenv_lower", "Parameters", greenv_lower)
# cv2.setTrackbarPos("greenh_upper", "Parameters", greenh_upper)
# cv2.setTrackbarPos("greens_upper", "Parameters", greens_upper)
# cv2.setTrackbarPos("greenv_upper", "Parameters", greenv_upper)


class Observer():
    def __init__(self):
        # if platform.machine() == 'x86_64':
        #     rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        # else:
        rospy.Subscriber("/video_source/raw", Image, self.img_cb)

        self.image = np.zeros((0, 0))  # subscriber image
        self.p_img = np.zeros((0, 0))  # processed image
        self.bridge = cv_bridge.CvBridge()  # cv_bridge
        # image publisher
        self.img_pub = rospy.Publisher("filtered_img", Image, queue_size=10)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.detected_color_pub = rospy.Publisher(
            "detected_color", String, queue_size=10)

        frec = 10  # frec var
        r = rospy.Rate(frec)  # Hz
        print("Node initialized {0}Hz".format(frec))
        while not rospy.is_shutdown():
            if self.image.size > 0:
                color = self.detect_color()
                self.detected_color_pub.publish(String(color))
                print("Detected color: {0}".format(color))
                cmd = Twist()
                if color == "RED":
                    cmd.linear.x = 0.0
                elif color == "GREEN":
                    cmd.linear.x = 0.3
                else:
                    cmd.linear.x = 0.0
                # self.cmd_pub.publish(cmd)
                # img to ROS Image msg
                # img_back = self.bridge.cv2_to_imgmsg(
                #     self.p_img, encoding="passthrough")
                # self.img_pub.publish(img_back)  # publish image
            r.sleep()

    def detect_color(self, show_img=False):
        image = cv2.GaussianBlur(self.image, (3, 3), 0) #self.image.copy()
        scale_percent = 50  # percent of original size
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dim = (width, height)


        # resize image
        resized = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
        # print(resized.shape)
        # cropped = resized[int(height*0.10):, ...]

        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
        # hsv[...,2] = cv2.multiply(hsv[..., 2], 0.6)
        # print(hsv.shape)

        # redh_lower = cv2.getTrackbarPos("redh_lower", "Parameters")
        # reds_lower = cv2.getTrackbarPos("reds_lower", "Parameters")
        # redv_lower = cv2.getTrackbarPos("redv_lower", "Parameters")
        # redh_upper = cv2.getTrackbarPos("redh_upper", "Parameters")
        # reds_upper = cv2.getTrackbarPos("reds_upper", "Parameters")
        # redv_upper = cv2.getTrackbarPos("redv_upper", "Parameters")

        # greenh_lower = cv2.getTrackbarPos("greenh_lower", "Parameters")
        # greens_lower = cv2.getTrackbarPos("greens_lower", "Parameters")
        # greenv_lower = cv2.getTrackbarPos("greenv_lower", "Parameters")
        # greenh_upper = cv2.getTrackbarPos("greenh_upper", "Parameters")
        # greens_upper = cv2.getTrackbarPos("greens_upper", "Parameters")
        # greenv_upper = cv2.getTrackbarPos("greenv_upper", "Parameters")

        # masks
        maskr_side = cv2.inRange(hsv, (redh_lower_side, reds_lower_side, redv_lower_side),
                            (redh_upper_side, reds_upper_side, redv_upper_side))
        # maskr_side = cv2.inRange(hsv, (redh_lower, reds_lower, redv_lower),
        #                     (redh_upper, reds_upper, redv_upper))
        maskr_front = cv2.inRange(hsv, (redh_lower_front, reds_lower_front, redv_lower_front),
                            (redh_upper_front, reds_upper_front, redv_upper_front))
        maskr = cv2.bitwise_or(maskr_front, maskr_side, mask=None)
        maskg = cv2.inRange(hsv, (greenh_lower, greens_lower, greenv_lower),
                            (greenh_upper, greens_upper, greenv_upper))  # green mask

        # erode
        kernel_e = np.ones((2, 2), np.uint8)
        erodedr = cv2.erode(maskr, kernel_e)
        erodedg = cv2.erode(maskg, kernel_e)

        kernel_d = np.ones((3, 3), np.uint8)
        dilatedg = cv2.dilate(erodedg, kernel_d)
        dilatedr = cv2.dilate(erodedr, kernel_d)



        largest_arear = self.largest_area(dilatedr)
        largest_areag = self.largest_area(dilatedg)



        # self.p_img = dilatedr


        if largest_arear > 5.0:
            return "RED"
        elif largest_areag > 5.0:
            return "GREEN"
        else:
            return
    def largest_area(self, img):
        cnts = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        largestArea = 0
        for cnt in cnts:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # count euclidian distance for each side of the rectangle
            sideOne = np.linalg.norm(box[0]-box[1])
            sideTwo = np.linalg.norm(box[0]-box[3])
            # count area of the rectangle
            area = sideOne*sideTwo
            # find the largest rectangle within all contours
            if area > largestArea:
                largestArea = area
            return largestArea

    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def cleanup(self):
        pass


if __name__ == "__main__":
    rospy.init_node("img_detector", anonymous=True)
    Observer()
