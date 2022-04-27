#!/usr/bin/env python
# Primero importamos las librerias
import rospy  
from geometry_msgs.msg import Twist


if __name__ =="__main__":
    # inicializamos el nodo
    rospy.init_node('cmd_vel_publisher', anonymous=True)
    # inicializamos el publicador al topico cmd_vel
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    # Tipo de mensaje utilizado
    robot_vel = Twist()
    # while hasta que se interrumpa el script
    while not rospy.is_shutdown():
        # velocidad para ir en contra de las manecillas
        robot_vel.angular.z = -0.1
        # publicar comando
        pub.publish(robot_vel)
        # sleep para garantizar el rate
        rate.sleep()
