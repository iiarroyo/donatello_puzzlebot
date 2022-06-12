#!/usr/bin/env python
import os
from turtle import right
import cv2
import rospy
import platform
import cv_bridge
import numpy as np
from numpy import dtype
from std_srvs.srv import Empty
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from matplotlib import cm, pyplot as plt
# vesl 0.2
# ki = 0.0015
# kp = 0.0035

# vel 0.25
# ki = 0.001
# kp = 0.005
# kd = 0.0
# vel 0.215
# ki = 0.0015
# kp = 0.0037
# kd = 0.0

ki = 0.0015
kp = 0.0037
kd = 0.0

frec = 20  # frec var
Ts = 1.0/float(frec)

K1 = kp + ki*Ts + kd/Ts
K2 = -kp - 2.0*kd/Ts
K3 = kd/Ts


class Observer():
    def __init__(self):
        if platform.machine() == 'x86_64':
            reset_simulation = rospy.ServiceProxy(
                '/gazebo/reset_simulation', Empty)
            reset_simulation()
            rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        else:
            rospy.Subscriber("/video_source/raw", Image, self.img_cb)
        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber("error_flag", String, self.error_flag_cb)

        self.image = None  # subscriber image
        self.p_img = np.zeros((0, 0))  # processed image
        self.bridge = cv_bridge.CvBridge()  # cv_bridge
        # image publisher
        self.img_pub = rospy.Publisher("filtered_img", Image, queue_size=10)
        self.cmd_pub = rospy.Publisher("vel_line", Twist, queue_size=10)
        self.line_pub = rospy.Publisher("line", Int32, queue_size=10)
        self.flag_pub = rospy.Publisher("line_detected", Bool, queue_size=10)

        # self.detected_color_pub = rospy.Publisher("detected_color", String, queue_size=10)

        r = rospy.Rate(frec)  # Hz
        print("Node initialized {0}Hz".format(frec))
        cmd = Twist()
        e = [0.0, 0.0, 0.0]  # error list
        u = [0.0, 0.0]
        line = 48
        while not rospy.is_shutdown():
            if self.image is None:
                print("image is None")
                continue
            line = self.detect_line()
            if line is not None:
                self.flag_pub.publish(True)
                self.line_pub.publish(line)
                print("idx: {0}".format(line))
                # cmd.linear.x = 0.085
                # e.insert(0, 48 - line)
                # e.pop()
                # u[0] = K1*e[0] + K2*e[1] + K3*e[2] + u[1]
                # cmd.angular.z = u[0]
                # u[1] = u[0]

                # print("angular", cmd.angular.z)
                # print("linear", cmd.linear.x)

            else:
                # cmd.linear.x = 0.0
                # cmd.angular.z = 0.0
                print("NO LINE DETECTED")
                self.flag_pub.publish(False)
            # print("idx: {0}".format(line))
            # self.cmd_pub.publish(cmd)
            # img to ROS Image msg
            if self.p_img is not None:
                img_back = self.bridge.cv2_to_imgmsg(
                    self.p_img, encoding="passthrough")
                self.img_pub.publish(img_back)  # publish image
            r.sleep()

    def detect_line(self, prev_line=None, plot=False):
        image = self.image.copy()
        scale_percent = 20  # percent of original size
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        img_height, img_width, _ = image.shape
        # resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
        gray_resized = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_resized_blur = cv2.GaussianBlur(gray_resized, (7, 7), 0)
        cropped = gray_resized_blur[int(
            img_height*0.70):, int(img_width*0.35):-int(img_width*0.35)]
        # cropped = gray_resized_blur[int(img_height*0.70):, :]
        ret, thresh = cv2.threshold(cropped, 100, 255, cv2.THRESH_BINARY)
        kernel = np.ones((6, 6), np.uint8)
        eroded = cv2.erode(~thresh, kernel)
        col_sum = np.sum(eroded, axis=0, dtype="int64")
        avg = np.mean(col_sum)

        left = eroded[:, :int(img_width*0.35)]
        right = eroded[:, -int(img_width*0.35):]

        left_sum = np.sum(left)
        right_sum = np.sum(right)
        # print("{0} {1}".format(left_sum, right_sum))
        # col_sum_l = col_sum[:int(len(col_sum)*0.5)]
        # col_sum_r = col_sum[int(len(col_sum)*0.5):]
        # dif = sum(col_sum_l)-sum(col_sum_r)

        self.p_img = eroded  # processed image

        #print("avg", avg)

        # if plot:
        # plt.figure(1)
        # x = np.arange(len(col_sum))
        # plt.plot(x, col_sum_l)
        # plt.show()
        # plt.figure(2)
        # plt.plot(x, col_sum_r)
        # plt.show()

        # return np.average(peaks)
        upper_pic_sum = np.sum(eroded[:int(img_height*0.50), :])
        lower_pic_sum = np.sum(eroded[int(img_height*0.50):, :])

        if avg < 400:
            return None
        # return avg
        idx = np.mean(np.where(col_sum > 1000))
        if idx < img_width//2:
            left = True
        elif idx > img_width//2:
            left = False

        return idx

    def group_cols(self, arr, win_size):
        for i in range(0, len(arr)//win_size, win_size):
            win_sum = 0
            for j in range(i, i+win_size, 1):
                win_sum += arr[j]
                arr[j] = 0
            arr[i] = win_sum
        return arr

    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return

    def error_flag_cb(self,msg):
        self.e = [0.0, 0.0, 0.0]  # error list
        self.u = [0.0, 0.0]

    def cleanup(self):
        last_cmd = Twist()
        self.cmd_pub.publish(last_cmd)


if __name__ == "__main__":
    rospy.init_node("img_detector", anonymous=True)
    Observer()
