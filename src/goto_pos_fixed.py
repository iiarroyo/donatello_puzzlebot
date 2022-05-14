#!/usr/bin/env python 
import rospy 
import time
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist  
from std_srvs.srv import Empty
from geometry_msgs.msg import Point

points = [(0,0), (1,1), (0,0)]
for i in range(1, len(points)):
    points[i] -= points[i+1]


class RobotPose(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        # reset simulation every time
        reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_simulation()
        self.points = [(0, 0), (1.5, -0.5), (2, 0), (0,0)]
        ###########  CONSTANTS  ############## 
        r=0.056                # wheel radius [m]
        L=0.185                 # wheel separation [m]
        self.d=0               # distance 
        self.theta=0           # angle
        self.wr=0              # right wheel vel measured
        self.wl=0              # left wheel vel measured
        self.cmd = Twist()     # robot vel

        K_v, K_w = 1.0, 1.4    # ctrl constants
        x_t, y_t = 1.1, 0.0    # goal coords
        theta, x, y = 0, 0, 0  # inital values
        
        #########   INIT PUBLISHERS   #########
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        
        ###########  SUBSCRIBERS  ############# 
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 

        ###########   INIT NODE   #############
        freq = 20
        rate = rospy.Rate(freq) #20Hz 
        Dt = 1/float(freq) #Dt is the time between one calculation and the next one
        print("Node initialized {0}hz".format(freq))
        cuenta = 0
        goals = self.get_goal()
        state = 0 # 0 straight, 1 turn 
        i = 0
        theta_t = 0
        theta_aux = 0
        while not rospy.is_shutdown():
            print("-----")
            
            # vels
            v = r*(self.wr+self.wl)/2.0
            w = r*(self.wr-self.wl)/L

            # pose
            x += v*Dt*np.cos(theta)
            y += v*Dt*np.sin(theta)
            theta += w*Dt
            theta = np.arctan2(np.sin(theta),np.cos(theta))

            # errors
            e_theta = np.arctan2(y_t, x_t) - theta
            e_d = np.sqrt(pow(x_t - x, 2) + pow(y_t - y, 2))

            # p control
            if state == 0:
                # print("Error dist:  ", e_d," Error angulo: ",e_theta)
                # print("x_t:  ", x_t," y_t: ",y_t)
                # print("x:  ", x, " y: ",y)

                if(e_d > 0.20):
                    v_out = 0.6#K_v * e_d
                    w_out = 0.0
                    # theta = 0
                else:
                    print("paraaa")
                    v_out = 0
                    w_out = 0
                    state = 1
                    i += 1
                    theta_t = np.arctan2(self.points[i][1] - self.points[i-1][1], self.points[i][0] - self.points[i-1][0]) - theta_t
            elif state == 1:
                print("theta_t", theta_t, "theta", theta_aux)
                theta_aux += w*Dt
                print("theta_t", theta_t, "theta", theta_aux)
                if(theta_aux < theta_t):
                    print("si")
                    v_out = 0
                    w_out = 0.3 #K_w * e_theta
                else:
                    v_out = 0
                    w_out = 0
                    theta_aux = 0
                    state = 2
            elif state == 2:
                time.sleep(1)
                print("Listo")
                x_t = self.points[i][0] 
                y_t = self.points[i][1]     
                state = 0
            # print("Distancia en x: ", x)
            # print("Distancia en y: ", y)
            # print("Rotacion en z: ", theta)

            # print("Vel lineal: ", v_out, " Vel ang: ",w_out)
            # # cmd_vel publish
            self.cmd.linear.x = v_out
            self.cmd.angular.z = w_out
            self.pub_cmd_vel.publish(self.cmd)
            i+=1
            rate.sleep() 

    def wl_cb(self, wl): 
        self.wl = wl.data
        
    def wr_cb(self, wr): 
        self.wr = wr.data

    def get_goal(self):
        for point in self.points:
            yield Point(point[0], point[1], 0)

        for point in reversed(self.points):
            yield Point(point[0], point[1], 0)
        
    def cleanup(self): 
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.pub_cmd_vel.publish(self.cmd)

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("GoToPos", anonymous=True) 
    RobotPose()
