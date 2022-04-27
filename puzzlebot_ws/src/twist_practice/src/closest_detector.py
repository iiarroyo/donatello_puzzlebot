#!/usr/bin/env python 
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
#This class will receive a number and an increment and it will publish the  
# result of adding number + increment in a recursive way. 
class ClosestDetectorClass(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 
        #********** INIT NODE **********### 
        r = rospy.Rate(1) #1Hz 
        print("Node initialized 1hz")
        while not rospy.is_shutdown(): 
            r.sleep()

    def laser_cb(self, msg): 
        ## This function receives a number  
        print(msg.angle_increment)
        print(msg.angle_min)
        print(min(msg.ranges))
        closest_range = min(msg.ranges)
        idx = msg.ranges.index(closest_range)
        closest_angle =  msg.angle_min + idx * msg.angle_increment
        print("Closest object distance: {0}".format(closest_range)) 
        print("Closest object direction: {0}".format(closest_angle))



    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        pass


############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("closest_detector", anonymous=True) 
    ClosestDetectorClass() 