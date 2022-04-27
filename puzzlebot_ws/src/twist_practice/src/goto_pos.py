#!/usr/bin/env python 
import rospy 
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist  

#This class will make the puzzlebot move following a square
class RobotPose(): 
    def __init__(self): 
        ############ CONSTANTS ################ 
        r=0.05 #wheel radius [m]
        L=0.18 #wheel separation [m]
        self.d=0 #distance 
        self.theta=0 #angle
        self.wr=0
        self.wl=0
        # vel.linear.x=0.3
        # vel.angular.z=0.2
        rospy.on_shutdown(self.cleanup) 
        ###******* INIT PUBLISHERS *******### 
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1) 
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 

        #********** INIT NODE **********### 
        freq=20
        rate = rospy.Rate(freq) #20Hz 
        Dt = 1.0/float(freq) #Dt is the time between one calculation and the next one
        print("Node initialized {0}hz".format(freq))
        state = 1 # 1 = moving forward, 2 = Turniing around, 3 = stop
        count = 0 # count the number of times the ronot has moved forward
        K_v, K_w = 0.5, 0.1  # ctrl constants
        x_t, y_t = 1.0, 1.0  # target coords
        theta, x, y = 0, 0, 0  # inital values
        while not rospy.is_shutdown():
            # vels
            v = r*(self.wr+self.wl)/2
            w = r*(self.wr-self.wl)/L
            # pose
            x += v*Dt*np.cos(theta)
            y += v*Dt*np.sin(theta)
            theta += w*Dt
            theta = theta%(2*np.pi)
            # errors
            e_theta = np.arctan2(y_t, x_t) - theta
            e_d = np.sqrt(pow(x_t - x, 2) + pow(y_t - y, 2))
            # p control
            print(e_theta, e_d)
            # v_out = K_v * e_d
            # w_out = K_w * e_theta
            # # cmd_vel publish
            # cmd  = Twist()
            # cmd.linear.x = v_out
            # cmd.angular.z = w_out
            # # publish msg
            # self.pub_cmd_vel.publish(cmd)
            rate.sleep() 
    def wl_cb(self, wl): 
        ## This function receives a number  
        self.wl = wl.data
        
    def wr_cb(self, wr): 
        ## This function receives a number. 
        self.wr = wr.data 
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        pass 
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("square", anonymous=True) 
    RobotPose()