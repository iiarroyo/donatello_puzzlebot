#!/usr/bin/env python
import os
import cv2
import rospy
import platform
import cv_bridge
import numpy as np
from numpy import dtype
from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from matplotlib import cm, pyplot as plt


class Observer():
    def __init__(self):
        if platform.machine() == 'x86_64':
            reset_simulation = rospy.ServiceProxy(
                            '/gazebo/reset_simulation', Empty)
            reset_simulation()
            rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        else:
            rospy.Subscriber("/video_source/raw", Image, self.img_cb)
        

        self.image = None  # subscriber image
        self.p_img = np.zeros((0, 0))  # processed image
        self.bridge = cv_bridge.CvBridge()  # cv_bridge
        # image publisher
        self.img_pub = rospy.Publisher("filtered_img", Image, queue_size=10)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        # self.detected_color_pub = rospy.Publisher("detected_color", String, queue_size=10)

        frec = 10  # frec var
        r = rospy.Rate(frec)  # Hz
        print("Node initialized {0}Hz".format(frec))
        cmd = Twist()
        prev_line = 0
        while not rospy.is_shutdown():
            if self.image is None:
                print("image is None")
                continue
            line = self.detect_line()
            if line is not None:
                print("Line: ", abs(48-line))
                if(abs(48-line)<10):
                    cmd.linear.x = (0.125)
                else:
                    cmd.linear.x = (0.05)

                # a_error = 0.003 * (40 - line)
                # if line > 0:
                #     cmd.angular.z = 0.15
                # elif line < 0:
                #     cmd.angular.z = -0.15
                # else:
                #     cmd.angular.z = 0.0
                cmd.angular.z = 0.003 * (48 - line)
                print("angular", cmd.angular.z )
                print("linear", cmd.linear.x)

            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            # img to ROS Image msg
            if self.p_img is not None:
                img_back = self.bridge.cv2_to_imgmsg(self.p_img, encoding="passthrough")
                self.img_pub.publish(img_back)  # publish image
            r.sleep()

    def detect_line(self, prev_line=None,plot=False):
        image = self.image.copy()
        scale_percent = 20 # percent of original size
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        img_height, img_width, _ = image.shape
        # resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
        gray_resized =  cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_resized_blur = cv2.GaussianBlur(gray_resized, (7, 7), 0)
        cropped = gray_resized_blur[int(img_height*0.70):, int(img_width*0.35):-int(img_width*0.35)]
        # print(cropped.shape)
        ret,thresh = cv2.threshold(cropped, 100, 255, cv2.THRESH_BINARY)
        # print(type(thresh))
        kernel = np.ones((6, 6), np.uint8)
        eroded = cv2.erode(~thresh, kernel)
        col_sum = np.sum(eroded, axis=0, dtype="int64")
        col_sum_l = col_sum[:int(len(col_sum)*0.5)]
        col_sum_r = col_sum[int(len(col_sum)*0.5):]
        #print(sum(col_sum_l))
        #print(sum(col_sum_r))
        dif=sum(col_sum_l)-sum(col_sum_r)
        # print("dif", dif)


        self.p_img = eroded  # processed image


        avg = np.mean(col_sum)
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
        if avg < 500:
            return None
        # return avg
        return np.mean(np.where(col_sum > 1000))

    def group_cols(self, arr, win_size):
        for i in range(0, len(arr)//win_size, win_size):
            win_sum = 0
            for j in range(i, i+win_size, 1):
                win_sum+=arr[j]
                arr[j] = 0
            arr[i] = win_sum
        return arr
            


    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return

    def cleanup(self):
        last_cmd = Twist()
        self.cmd_pub.publish(last_cmd)
    

if __name__ == "__main__":
    rospy.init_node("img_detector", anonymous=True)
    Observer()

