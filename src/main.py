import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


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
        self.cmd_vel = Twist()

        rospy.Subscriber("/line_idx", Int32, self.line_cb)
        rospy.Subscriber("/color", String, self.line_cb)
        rospy.Subscriber("/sign", String, self.line_cb)
        rospy.Subscriber("wl", Float32, self.wl_cb)
        rospy.Subscriber("wr", Float32, self.wr_cb)

        self.line_idx = None
        self.color = None
        self.sign = None

        self.current_state = STOP
        self.future_state = None
        self.line_detected = True
        self.linear_velocity = 0.1
        self.angular_velocity = 0.1
        self.freq = 20
        self.rate = rospy.Rate(self.freq)  # 20Hz
        self.Dt = 1/float(self.freq)

        self.kv = 0.1
        self.kw = 0.1
        self.r = 0.05                 # wheel radius [m]
        self.L = 0.18                 # wheel separation [m]

    def go_forward(self):
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        while self.state == FORWARD:
            cmd = self.goto_pos(0.10, 0.0)
            self.cmd_vel_pub.publish(cmd)

    def turn_right(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.1
        self.cmd_vel_pub.publish(self.cmd_vel)

    def turn_left(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = -0.1
        self.cmd_vel_pub.publish(self.cmd_vel)

    def stop(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)

    def follow_line(self, line_idx):
        if self.line_idx is None:
            self.current_state = self.future_state
            self.future_state = None
        self.cmd_vel.linear.x = 0.1
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)

    # -------------------------------------------------------------------------

    def increase_velocity(self):
        self.cmd_vel.linear.x = 0.2
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)

    def decrease_velocity(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)

    def goto_pos(self, x_target, y_target):
        v = self.r*(self.wr+self.wl)/2
        w = self.r*(self.wr-self.wl)/self.L
        self.x += v*self.Dt*np.cos(self.theta)
        self.y += v*self.Dt*np.sin(self.theta)
        self.theta += w*self.Dt
        e_theta = np.arctan2(y_target, x_target) - self.theta
        e_d = np.sqrt(pow(x_target - self.x, 2) + pow(y_target - self.y, 2))
        self.cmd_vel.linear.x = self.linear_velocity
        self.cmd_vel.angular.z = self.angular_velocity

    # -------------------------------------------------------------------------

    def wl_cb(self, msg):
        self.wl = msg.data

    def wr_callback(self, msg):
        self.wr = msg.data

    def line_cb(self, msg):
        self.line_idx = msg.data
        # TODO: if no line is detected, return None

    def sign_cb(self, msg):
        self.sign = msg.data
        if self.sign == "turn_left":
            self.next_state = TURN_LEFT
        elif self.sign == "turn_right":
            self.next_state = TURN_RIGHT

    def color_cb(self, msg):
        self.color = msg.data

    def execute(self):
        if self.state == STOP:
            self.stop()
        elif self.state == FORWARD:
            self.go_forward()
        elif self.state == BACKWARD:
            self.go_backward()
        elif self.state == TURN_LEFT:
            self.turn_left()
        elif self.state == TURN_RIGHT:
            self.turn_right()
        elif self.state == FOLLOW_LINE:
            self.follow_line()
        else:
            self.stop()

    def main(self):
        while not rospy.is_shutdown():
            if self.color == "red" or self.sign == "stop":
                self.current_state = STOP
            self.execute()

            # self.execute(state)


if __name__ == '__main__':
    rospy.init_node('control_node', anonymous=True)
    rospy.loginfo("Control node initialized")
    control = Control()
    control.main()
