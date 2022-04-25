#!/usr/bin/env python 
import rospy 
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist  

class RobotPose(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 

        ###########  CONSTANTS  ############## 
        r=0.05                 # wheel radius [m]
        L=0.18                 # wheel separation [m]
        self.d=0               # distance 
        self.theta=0           # angle
        self.wr=0              # right wheel vel measured
        self.wl=0              # left wheel vel measured
        self.cmd = Twist()     # robot vel

        K_v, K_w = 0.1, 1.5    # ctrl constants
        x_t, y_t = -1.0, 0.0    # goal coords
        theta, x, y = 0, 0, 0  # inital values
    
        #########   INIT PUBLISHERS   #########
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1) 
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        
        ###########  SUBSCRIBERS  ############# 
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 

        ###########   INIT NODE   #############
        freq = 20
        rate = rospy.Rate(freq) #20Hz 
        Dt = 1/float(freq) #Dt is the time between one calculation and the next one
        print("Node initialized {0}hz".format(freq))
    
        while not rospy.is_shutdown():
            # vels
            v = r*(self.wr+self.wl)/2
            w = r*(self.wr-self.wl)/L

            # pose
            x += v*Dt*np.cos(theta)
            y += v*Dt*np.sin(theta)
            theta += w*Dt
            if theta > np.pi:
                theta = theta - 2*np.pi
            elif theta < -np.pi:
                theta = theta + 2*np.pi

            # errors
            e_theta = np.arctan2(y_t, x_t) - theta
            e_d = np.sqrt(pow(x_t - x, 2) + pow(y_t - y, 2))

            # p control
            if(e_d > 0.2):
                v_out = K_v * e_d
                w_out = K_w * e_theta
                if(w_out>1):
                   w_out = 1
            else:
                v_out = 0
                w_out = 0

            print("Error dist: ", e_d," Error angulo: ",e_theta)
            print("Vel lineal: ", v_out, " Vel ang: ",w_out)
            # cmd_vel publish
            self.cmd.linear.x = v_out
            self.cmd.angular.z = w_out
            self.pub_cmd_vel.publish(self.cmd)

            rate.sleep() 

    def wl_cb(self, wl): 
        self.wl = wl.data
        
    def wr_cb(self, wr): 
        self.wr = wr.data 
        
    def cleanup(self): 
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.pub_cmd_vel.publish(self.cmd)

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("square", anonymous=True) 
    RobotPose()