#!/usr/bin/env python 
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class RobotControl(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        self.pub= rospy.Publisher('cmd_vel', Twist, queue_size=10) 
        rospy.Subscriber("puzzlebot_cmd", String, self.set_vel) 
        r = rospy.Rate(1) #1Hz
        w = 0.5  # turn speed
        vel = 0.5  # velocity

        # Defining movements
        self.move_forward = Twist()
        self.move_forward.linear.x = vel

        self.move_back = Twist()
        self.move_back.linear.x = -vel

        self.turn_left = Twist()
        self.turn_left.angular.z = w

        self.turn_right = Twist()
        self.turn_right.angular.z = -w

        self.circle = Twist()
        self.circle.angular.z = -w
        self.circle.linear.x = vel

        self.stop = Twist()  # all values initialized in 0

        # dict with possible commands
        self.cmds = {"Move forward":self.move_forward,
                     "Move back":self.move_back,
                     "Turn left":self.turn_left,
                     "Turn right":self.turn_right,
                     "Stop":self.stop,
                     "Circle":self.circle }

        # puzzlebot cmd
        self.current_cmd = Twist()

        while not rospy.is_shutdown():
            self.pub.publish(self.current_cmd)
            print(self.current_cmd)
            r.sleep()

    def set_vel(self, cmd):
        """
        modifies puzzlebot vel when called
        """
        # access dict
        cmd2publish = self.cmds[cmd.data]
        if cmd2publish:  # if read value is valid
            self.current_cmd = cmd2publish

    def cleanup(self):
        """
        set puzzzlebot vel to 0
        """
        print(r"Shutting down ...")
        self.pub.publish(self.cmds["Stop"])
        # sleep for appreciating puzzlebot stop
        time.sleep(5)


if __name__ == "__main__": 
    rospy.init_node("adder_node", anonymous=True) 
    RobotControl() 