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

dire = os.path.join(os.path.sep,"home", "iiarroyo", "Pictures")
# reset_simulation = rospy.ServiceProxy(
#                 '/gazebo/reset_simulation', Empty)
# reset_simulation()

class Observer():
    def __init__(self):
        # if platform.machine() == 'x86_64':
        #     rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        # else:
        rospy.Subscriber("/video_source/raw", Image, self.img_cb)
        

        self.image = None  # subscriber image
        self.p_img = np.zeros((0, 0))  # processed image
        self.bridge = cv_bridge.CvBridge()  # cv_bridge
        # image publisher
        self.img_pub = rospy.Publisher("filtered_img", Image, queue_size=10)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        frec = 10  # frec var
        r = rospy.Rate(frec)  # Hz
        print("Node initialized {0}Hz".format(frec))
        i=0
        os.chdir(dire)
        while not rospy.is_shutdown():
            if self.image is None:
                print("image is None")
                continue
            print(self.image.shape)
            cv2.imshow("hola", self.image)
            key = cv2.waitKey(1) & 0xff
            print(key)
            if key == ord('q'):
                print(os.getcwd())
                cv2.imwrite(str(i)+".png", self.image)
                i+=1
            r.sleep()


    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return



if __name__ == "__main__":
    rospy.init_node("img_detector", anonymous=True)
    Observer()

