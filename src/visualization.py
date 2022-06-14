#!/usr/bin/env python 

import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv_bridge
import cv2

STOP = 0
FORWARD = 1
BACKWARD = 2
TURN_LEFT = 3
TURN_RIGHT = 4
FOLLOW_LINE = 5
RED_LIGHT = 6

class Visualization():
    def __init__(self):
        
    # ---------------------     SUSCRIBERS       -----------------------------

        rospy.Subscriber("/line_detected", Bool, self.line_det_cb)
        rospy.Subscriber("/line", Int32, self.line_cb)
        rospy.Subscriber("/start", Bool, self.start_cb)
        rospy.Subscriber("/detected_color", String, self.color_cb)
        rospy.Subscriber("/sign", String, self.sign_cb)
        rospy.Subscriber("cropped_image", Image, self.img_cb)
        rospy.Subscriber("cmd_vel", Twist, self.vel_cb)
        rospy.Subscriber("current_state", String, self.state_cb)

    # ---------------------     CONSTANTS       ------------------------------
  
        self.bridge = cv_bridge.CvBridge()
        self.start_robot = False
        self.line_detected = False
        self.line_idx = 48
        self.color = "Unknow"
        self.sign = "Waiting"
        self.current_state = "Waiting"
        self.vel = Twist()
        self.image = np.zeros((0, 0))
        self.freq = 20
        self.rate = rospy.Rate(self.freq)  
        self.Dt = 1/float(self.freq)

    # ---------------------     CALLBACKS       -------------------------------
    
    def line_det_cb(self, msg):
        self.line_detected = msg.data

    def line_cb(self, msg):
        self.line_idx = msg.data

    def start_cb(self, msg):
        self.start_robot = msg.data

    def color_cb(self, msg):
        self.color = msg.data

    def sign_cb(self, msg):
        self.sign = msg.data
    
    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def vel_cb(self, msg):
        self.vel = msg

    def state_cb(self,msg):
       self.current_state = msg.data

    # ------------------- VISUALS ----------------------------------------

    def main(self):

        while not rospy.is_shutdown():  

            if(self.image is not None):
                if(self.start_robot == True):
                    #cv2.imshow("Cropped Image",self.image)
                    print("\n----------------\n")   
                    print("Current State: {0}".format(self.current_state))
                    print("Last signal: {0}".format(self.sign))
                    print("Traffic Light Color: {0}".format(self.color))
                    print("is Line Detected? {0}".format("YES" if self.line_detected else "NO"))
                    print("Line idx: {0}".format(self.line_idx))
                    print("Velocities: {0}".format(self.vel))
                else:
                    print("Waiting for start signal...")

                # if cv2.waitKey(1) & 0xFF is ord('q'):
                #     cv2.destroyAllWindows()
                #     print("Stop programm and close all windows")
                #     break

            self.rate.sleep()  

# ------------------- MAIN -----------------------------------------------

if __name__ == '__main__':
    rospy.init_node('Visualization_node', anonymous=True)
    rospy.loginfo("Node initialized")
    visuals = Visualization()
    visuals.main()