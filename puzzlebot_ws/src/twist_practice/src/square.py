#!/usr/bin/env python 
import rospy 
from geometry_msgs.msg import Twist  
from std_msgs.msg import Float32
import numpy as np

#This class will make the puzzlebot move following a square
class SquareClass(): 
    def __init__(self): 
        ############ CONSTANTS ################ 
        r=0.05 #wheel radius [m]
        L=0.18 #wheel separation [m]
        self.d=0 #distance 
        self.theta=0 #angle
        vel=Twist() #Robot's speed
        self.wr=0
        self.wl=0
        vel.linear.x=0.3
        vel.angular.z=0.2
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
        Dt =1/20.0 #Dt is the time between one calculation and the next one
        print("Node initialized 20hz")
        state = 1 # 1 = moving forward, 2 = Turniing around, 3 = stop
        count = 0 # count the number of times the ronot has moved forward
        while not rospy.is_shutdown(): 
            v=r*(self.wr+self.wl)/2
            w=r*(self.wr-self.wl)/L
            self.d=v*Dt+self.d
            self.theta=w*Dt+self.theta
            # moving forward
            if state == 1:
                print("Moving forward")
                vel.linear.x = 0.2
                vel.angular.z = 0
                if self.d > 1:
                    self.d = 0.0
                    state = 2  # change to turn around state
            # turn around
            elif state == 2:
                print("turn around")
                vel.linear.x = 0
                vel.angular.z = 0.4
                if self.theta > np.pi/2:
                    if count < 3:
                        state = 1
                        count += 1
                        self.theta = 0.0
                    else:
                        state = 3
            #stop
            elif state == 3:
                print("stop")
                vel.linear.z = 0
                vel.angular.z = 0
            
            self.pub_cmd_vel.publish(vel) #publish the robot's speed 
            print(v)
            print(w)
            print(self.d)
            print(self.theta)
            print(Dt)
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
    SquareClass()