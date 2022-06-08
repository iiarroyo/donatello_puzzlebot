#!/usr/bin/env python 

import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

# STATES
STOP = 0
FORWARD = 1
BACKWARD = 2
TURN_LEFT = 3
TURN_RIGHT = 4
FOLLOW_LINE = 5


class Control():
    def __init__(self):
        
    # ---------------------     PUBLISHERS       -----------------------------

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    # ---------------------     SUSCRIBERS       -----------------------------

        rospy.Subscriber("/start", String, self.start_cb)
        rospy.Subscriber("/line", Int32, self.line_cb)
        rospy.Subscriber("/color", String, self.color_cb)
        rospy.Subscriber("/sign", String, self.sign_cb)
        rospy.Subscriber("/vel_line", Twist, self.vel_cb)


    # ---------------------     CONSTANTS       ------------------------------

        self.line_idx = None
        self.color = None
        self.sign = None
        self.start_robot = "False"
        self.current_state = STOP
        self.vel_line = Twist()
        self.cmd_vel = Twist()

        self.freq = 20
        self.rate = rospy.Rate(self.freq)  # 20Hz
        self.Dt = 1/float(self.freq)

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

    # ------------------- STATE MACHINE ----------------------------------------

    def main(self):
        while not rospy.is_shutdown():
            
            if(self.start_robot == "True"):
                self.current_state = FOLLOW_LINE
            
            if(self.current_state == FOLLOW_LINE):
                if(self.sign == "GoAhead" and self.line_idx == -1):
                    self.current_state = FORWARD
                elif(self.sign == "TurnRight" and self.line_idx == -1):
                    self.current_state = TURN_RIGHT
                elif(self.sign == "TurnLeft" and self.line_idx == -1):
                    self.current_state = TURN_LEFT
                elif(self.sign == "Stop" or self.color == "Red"):
                    self.current_state = STOP
                else:
                    self.cmd_vel = self.vel_line
                    print("Following Line")

            if(self.current_state == FORWARD):
                print("Going forward")
                time.sleep(5)
                self.current_state = FOLLOW_LINE

            if(self.current_state == TURN_RIGHT):
                print("Turning Right")
                time.sleep(5)
                self.current_state = FOLLOW_LINE

            if(self.current_state == TURN_LEFT):
                print("Turning Left")
                time.sleep(5)
                self.current_state = FOLLOW_LINE

            if(self.current_state == STOP):
                if(self.color == "Green"):
                    self.current_state == FOLLOW_LINE
                else:
                    self.cmd_vel.linear.x = 0
                    self.cmd_vel.linear.z = 0
                    print("Stopped")
            
            self.cmd_vel_pub.publish(self.cmd_vel)

            self.rate.sleep()  

# ------------------- MAIN -----------------------------------------------

if __name__ == '__main__':
    rospy.init_node('control_node', anonymous=True)
    rospy.loginfo("Control node initialized")
    control = Control()
    control.main()

