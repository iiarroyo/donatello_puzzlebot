#!/usr/bin/env python 

import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time

# STATES
STOP = 0
FORWARD = 1
BACKWARD = 2
TURN_LEFT = 3
TURN_RIGHT = 4
FOLLOW_LINE = 5
RED_LIGHT = 6


class Control():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
    # ---------------------     PUBLISHERS       -----------------------------

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    # ---------------------     SUSCRIBERS       -----------------------------

        rospy.Subscriber("/line_detected", Bool, self.line_det_cb)
        rospy.Subscriber("/start", Bool, self.start_cb)
        rospy.Subscriber("/line", Int32, self.line_cb)
        rospy.Subscriber("/detected_color", String, self.color_cb)
        rospy.Subscriber("/sign", String, self.sign_cb)
        # rospy.Subscriber("wl", Float32, self.wl_cb)
        # rospy.Subscriber("wr", Float32, self.wr_cb)
        
    # ---------------------     CONSTANTS       ------------------------------

        self.line_idx = 48
        self.line_detected = False
        self.line = 48
        self.curr_sign = "None"
        self.color = None
        self.sign = None
        self.start_robot = False
        self.current_state = None
        self.cmd_vel = Twist()
        self.wl = 0.0
        self.wr = 0.0
        self.r = 0.05                 
        self.L = 0.18   
        self.LimitVel = True              
        self.freq = 20
        self.rate = rospy.Rate(self.freq)  # 20Hz
        self.Dt = 1/float(self.freq)

    # ---------------------     CALLBACKS       -------------------------------
    
    def start_cb(self, msg):
        self.start_robot = msg.data

    def line_det_cb(self, msg):
        self.line_detected = msg.data
    
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

    # def forward_move(self, dist):
    #     x = 0
    #     vel_forward = Twist()
    #     while(x<dist):
    #         print("x",x)
    #         #print("wl",self.wl,"  wr",self.wr)
    #         v = self.r*(self.wr+self.wl)/2
    #         x += v*self.Dt*np.cos(0)

    #         vel_forward.linear.x = 0.1
    #         vel_forward.angular.z = 0
    #         self.cmd_vel_pub.publish(vel_forward)
    #         #rate

    # def turn(self, rotation_goal,cw=True):
    #     x = 0
    #     theta = 0
    #     vel_forward = Twist()
    #     e_theta = rotation_goal - theta
    #     while(abs(e_theta)>0.05):
    #         v = self.r*(self.wr+self.wl)/2
    #         w = self.r*(self.wr-self.wl)/self.L
    #         x += v*self.Dt*np.cos(theta)
    #         theta += w*self.Dt
    #         e_theta = rotation_goal - theta

    #         vel_angular = 0.2 if cw else -0.2
    #         vel_forward.linear.x = 0
    #         vel_forward.angular.z = vel_angular
    #         self.cmd_vel_pub.publish(vel_forward)


    # ------------------- STATE MACHINE ----------------------------------------

    def main(self):
        while not rospy.is_shutdown():
            
            self.curr_sign = self.sign
            print(self.curr_sign)

            if(self.curr_sign == "NoLimitVel"):
                self.LimitVel = False
        
            if(self.start_robot == True):
                self.current_state = FOLLOW_LINE
                self.start_robot = False
            
            if(self.current_state == FOLLOW_LINE):
                if(self.color == "RED" and self.line_detected == False):
                    self.current_state = RED_LIGHT
                elif(self.curr_sign == "Stop"):
#                    self.sign = "None"
                    self.current_state = STOP
                elif(self.curr_sign == "GoAhead" and self.line_detected == False):
 #                   self.sign = "None"
                    self.current_state = FORWARD
                elif(self.curr_sign == "TurnRight" and self.line_detected == False):
  #                  self.sign = "None"
                    self.current_state = TURN_RIGHT
                elif(self.curr_sign == "TurnLeft" and self.line_detected == False):
   #
   #                  self.sign = "None"
                    self.current_state = TURN_LEFT
                else:
                    if(self.LimitVel == True):
                        self.line = self.line_idx
                        self.cmd_vel.linear.x = 0.085 * 0.75
                        self.cmd_vel.angular.z = 0.003*(48-self.line)
                    else:
                        self.line = self.line_idx
                        self.cmd_vel.linear.x = 0.085 * 1.50
                        self.cmd_vel.angular.z = 0.003*(48-self.line)
                    self.cmd_vel_pub.publish(self.cmd_vel)
                    print("Following Line")

            if(self.current_state == FORWARD):
                print("Going forward")
                self.cmd_vel.linear.x = 0.1
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)
                time.sleep(5)
                print("Went forward")
                self.current_state = FOLLOW_LINE

            if(self.current_state == TURN_RIGHT):
                print("Turning Right")               
                #Enfrente
                self.cmd_vel.linear.x = 0.1
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)
                time.sleep(2.8)
                #Vuelta
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = -0.1
                self.cmd_vel_pub.publish(self.cmd_vel)
                time.sleep(3.3)
                #Derecha
                self.cmd_vel.linear.x = 0.1
                self.cmd_vel.angular.z = 0
                self.cmd_vel_pub.publish(self.cmd_vel)
                time.sleep(2)
                print("Turned right")
                self.current_state = FOLLOW_LINE

            if(self.current_state == TURN_LEFT):
                print("Turning Left")
                self.current_state = FOLLOW_LINE

            if(self.current_state == STOP):
                if(self.color != "GREEN"):
                    self.cmd_vel.linear.x = 0
                    self.cmd_vel.angular.z = 0
                    self.cmd_vel_pub.publish(self.cmd_vel)
                    print("Stop Sign")
                elif (self.color == "GREEN"):
                    self.current_state = FOLLOW_LINE

            if(self.current_state == RED_LIGHT):
                if(self.color != "GREEN"):
                    self.cmd_vel.linear.x = 0
                    self.cmd_vel.angular.z = 0
                    self.cmd_vel_pub.publish(self.cmd_vel)
                    print("Red Light")
                elif (self.color == "GREEN"):
                    self.current_state = FOLLOW_LINE
                
            self.rate.sleep()  

    def cleanup(self):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel)

# ------------------- MAIN -----------------------------------------------

if __name__ == '__main__':
    rospy.init_node('control_node', anonymous=True)
    rospy.loginfo("Control node initialized")
    control = Control()
    control.main()











# --------- borradores

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

    # def wl_cb(self, msg):
    #     self.wl = msg.data

    # def wr_cb(self, msg):
    #     self.wr = msg.data

        
        #self.cmd_vel = Twist()
        #self.kv = 0.1
        #self.kw = 0.1
        #self.r = 0.05                 # wheel radius [m]
        #self.L = 0.18                 # wheel separation [m]

        #self.linear_velocity = 0.1
        #self.angular_velocity = 0.1

        #rospy.Subscriber("wl", Float32, self.wl_cb)
        #rospy.Subscriber("wr", Float32, self.wr_cb)
