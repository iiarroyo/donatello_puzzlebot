#!/usr/bin/env python
import sys
import time
import rospy
import platform
import numpy as np
from std_srvs.srv import Empty
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String

class RobotPose():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # reset simulation every time
        if platform.machine() == 'x86_64':
            reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            reset_simulation()
        j = 0
        self.points = np.array([
                                [0.0, 0.0],
                                [1.0, 0.0],
                                [1.5, -0.5],
                                [2.0, 0.0]])
        self.goals = self.get_goal()
        goal = self.goals.next()
        prev_goal = None
        ###########  CONSTANTS  ##############
        r=0.05                 # wheel radius [m]
        L=0.18                 # wheel separation [m]
        self.d=0               # distance 
        self.theta=0           # angle
        self.wr=0              # right wheel vel measured
        self.wl=0              # left wheel vel measured
        self.cmd = Twist()     # robot vel
        self.color = None

        K_v, K_w = 0.8, 0.6    # ctrl constants
        # x_t, y_t = points[j][0], points[j][1]   # goal coords
        theta, x, y = 0, 0, 0  # inital values
        theta_aux, x_aux, y_aux = 0,0, 0

        #########   INIT PUBLISHERS   #########
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        ###########  SUBSCRIBERS  #############
        rospy.Subscriber("wl", Float32, self.wl_cb)
        rospy.Subscriber("wr", Float32, self.wr_cb)
        rospy.Subscriber("/detected_color", String, self.color_cb)

        ###########   INIT NODE   #############
        freq = 20
        rate = rospy.Rate(freq) #20Hz
        Dt = 1/float(freq) #Dt is the time between one calculation and the next one
        print("Node initialized {0}hz".format(freq))
        while self.color != "GREEN": pass # wait until green is detected

        while not rospy.is_shutdown():
            if self.color == "RED": break
            # vels
            v = r*(self.wr+self.wl)/2
            w = r*(self.wr-self.wl)/L

            # pose
            x += v*Dt*np.cos(theta)
            y += v*Dt*np.sin(theta)
            theta += w*Dt
            x_aux += v*Dt*np.cos(theta_aux)
            y_aux += v*Dt*np.sin(theta_aux)
            theta_aux += w*Dt
            theta = np.arctan2(np.sin(theta),np.cos(theta))

            # errors
            e_theta = np.arctan2(goal.y, goal.x) - theta
            e_d = np.sqrt(pow(goal.x - x, 2) + pow(goal.y - y, 2))

            v_max = 0.4
            w_max = 1.7

            # p control
            if(abs(e_theta) > 0.1):
                    v_out = 0.0
                    w_out = K_w * e_theta
            elif(e_d > 0.07):
                    v_out = K_v * e_d
                    w_out = K_w * e_theta
            else:
                # x_t, y_t = -1.0, -1.0
                x, y = 0,0
                v_out = 0.0
                w_out = 0.0
                prev_goal = goal
                goal = self.goals.next()
                print(goal.x, goal.y)
                # x_t = points[j][0] 
                # y_t = points[j][1]
            # print(x_aux, y_aux)
            # print("Error dist: ", e_d," Error angulo: ",e_theta)
            # print("Vel lineal: ", v_out, " Vel ang: ",w_out)
            # cmd_vel publish
            self.cmd.linear.x = v_out
            self.cmd.angular.z = w_out
            self.pub_cmd_vel.publish(self.cmd)
            rate.sleep()

    def wl_cb(self, wl):
        self.wl = wl.data

    def wr_cb(self, wr):
        self.wr = wr.data
    
    def color_cb(self, color):
        self.color = color.data

    def get_goal(self):
        for i in range(1, len(self.points)):
            yield Point(self.points[i, 0] - self.points[i - 1, 0],  # x
                        self.points[i, 1] - self.points[i - 1, 1],  # y
                         0)                                         # z
        
        for i in range(len(self.points)-2, -1, -1):
            yield Point(self.points[i, 0] - self.points[i + 1, 0],  # x
                        self.points[i, 1] - self.points[i + 1, 1],  # y
                         0)                                         # z


    def cleanup(self):
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.pub_cmd_vel.publish(self.cmd)

############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("Navigation node", anonymous=True)
    RobotPose()
