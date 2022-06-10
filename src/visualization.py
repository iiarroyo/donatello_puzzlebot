#!/usr/bin/env python 

import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import cv2




class Visualization():
    def __init__(self):
        
    # ---------------------     PUBLISHERS       -----------------------------

        #self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    # ---------------------     SUSCRIBERS       -----------------------------

        rospy.Subscriber("/start", String, self.start_cb)
        rospy.Subscriber("/line", Int32, self.line_cb)
        rospy.Subscriber("/detected_color", String, self.color_cb)
        rospy.Subscriber("/sign", String, self.sign_cb)
        #rospy.Subscriber("/detection_image", , self.img_cb)


    # ---------------------     CONSTANTS       ------------------------------

#     Current State: 
#     Last Signal:
#     Traffic Light:

        self.line_idx = None
        self.color = None
        self.sign = None
        self.start_robot = "False"
        self.current_state = None
        self.vel_line = Twist()
        self.cmd_vel = Twist()
        self.wl = 0.0
        self.wr = 0.0
        self.r = 0.05                 
        self.L = 0.18   
        self.LimitVel = True              

        self.freq = 20
        self.rate = rospy.Rate(self.freq)  # 20Hz
        self.Dt = 1/float(self.freq)

    #####

        img = np.ones((30,30,1))
        crop = np.random.random((10, 10))
        crop = np.resize(crop,(10,10,1))
        states = np.zeros((10,20,1))
        down_image = np.concatenate((crop, states), axis=1)
        self.full_image = np.concatenate((img, down_image), axis=0)
        self.window = cv2.namedWindow("Donatello",cv2.WINDOW_NORMAL)
        

    ####


    # ---------------------     CALLBACKS       -------------------------------
    
    def start_cb(self, msg):
        self.start_robot = msg.data

    def line_cb(self, msg):
        self.line_idx = msg.data

    def sign_cb(self, msg):
        self.sign = msg.data

    def color_cb(self, msg):
        self.color = msg.data

    def vel_cb(self, msg):
        self.vel_line = msg

    def wl_cb(self, msg):
        self.wl = msg.data

    def wr_cb(self, msg):
        self.wr = msg.data


    # -------------------   FUNCTIONS   ----------------------------------------

    # ------------------- VISUALS ----------------------------------------

    def main(self):
        while not rospy.is_shutdown():
            cv2.imshow("Donatello",self.full_image)
            if cv2.waitKey(1) & 0xFF is ord('q'):
                cv2.destroyAllWindows()
                print("Stop programm and close all windows")
                break
            self.rate.sleep()  

   

# ------------------- MAIN -----------------------------------------------

if __name__ == '__main__':
    rospy.init_node('Visualization_node', anonymous=True)
    rospy.loginfo("Node initialized")
    visuals = Visualization()
    visuals.main()


