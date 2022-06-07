#!/usr/bin/env python 

import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

# States
STOP = 0
FORWARD = 1
BACKWARD = 2
TURN_LEFT = 3
TURN_RIGHT = 4
FOLLOW_LINE = 5


class Control():
    def __init__(self):
        
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("/start", String, self.start_cb)
        rospy.Subscriber("/line", Int32, self.line_cb)
        rospy.Subscriber("/color", String, self.color_cb)
        rospy.Subscriber("/sign", String, self.sign_cb)
        
        #rospy.Subscriber("wl", Float32, self.wl_cb)
        #rospy.Subscriber("wr", Float32, self.wr_cb)

        self.line_idx = None
        self.color = None
        self.sign = None
        self.start_robot = "False"
        self.current_state = STOP

        #self.linear_velocity = 0.1
        #self.angular_velocity = 0.1
        self.freq = 20
        self.rate = rospy.Rate(self.freq)  # 20Hz
        self.Dt = 1/float(self.freq)
        
        #self.cmd_vel = Twist()
        #self.kv = 0.1
        #self.kw = 0.1
        #self.r = 0.05                 # wheel radius [m]
        #self.L = 0.18                 # wheel separation [m]


    # ---------------------     CALLBACKS       ---------------------------------
    
    def start_cb(self, msg):
        self.start_robot = msg.data

    # def wl_cb(self, msg):
    #     self.wl = msg.data

    # def wr_cb(self, msg):
    #     self.wr = msg.data

    def line_cb(self, msg):
        self.line_idx = msg.data
        # TODO: if no line is detected, return None

    def sign_cb(self, msg):
        self.sign = msg.data

    def color_cb(self, msg):
        self.color = msg.data


    # ------------------- MAIN --------------------------------------------------

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
                    print("Stopped")
                
            self.rate.sleep()  


if __name__ == '__main__':
    rospy.init_node('control_node', anonymous=True)
    rospy.loginfo("Control node initialized")
    control = Control()
    control.main()




        # def execute(self):
    #     if self.state == STOP:
    #         self.stop()
    #     elif self.state == FORWARD:
    #         self.go_forward()
    #     elif self.state == BACKWARD:
    #         self.go_backward()
    #     elif self.state == TURN_LEFT:
    #         self.turn_left()
    #     elif self.state == TURN_RIGHT:
    #         self.turn_right()
    #     elif self.state == FOLLOW_LINE:
    #         self.follow_line()
    #     else:
    #         self.stop()


     # def go_forward(self):
    #     self.x, self.y, self.theta = 0.0, 0.0, 0.0
    #     while self.state == FORWARD:
    #         cmd = self.goto_pos(0.10, 0.0)
    #         self.cmd_vel_pub.publish(cmd)

    # def turn_right(self):
    #     self.cmd_vel.linear.x = 0.0
    #     self.cmd_vel.angular.z = 0.1
    #     self.cmd_vel_pub.publish(self.cmd_vel)

    # def turn_left(self):
    #     self.cmd_vel.linear.x = 0.0
    #     self.cmd_vel.angular.z = -0.1
    #     self.cmd_vel_pub.publish(self.cmd_vel)

    # def stop(self):
    #     self.cmd_vel.linear.x = 0.0
    #     self.cmd_vel.angular.z = 0.0
    #     self.cmd_vel_pub.publish(self.cmd_vel)

    # def follow_line(self, line_idx):
    #     if self.line_idx is None:
    #         self.current_state = self.future_state
    #         self.future_state = None
    #     self.cmd_vel.linear.x = 0.1
    #     self.cmd_vel.angular.z = 0.0
    #     self.cmd_vel_pub.publish(self.cmd_vel)

    # # -------------------------------------------------------------------------

    # def increase_velocity(self):
    #     self.cmd_vel.linear.x = 0.2
    #     self.cmd_vel.angular.z = 0.0
    #     self.cmd_vel_pub.publish(self.cmd_vel)

    # def decrease_velocity(self):
    #     self.cmd_vel.linear.x = 0.0
    #     self.cmd_vel.angular.z = 0.0
    #     self.cmd_vel_pub.publish(self.cmd_vel)