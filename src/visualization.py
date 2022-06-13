#!/usr/bin/env python 

import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import cv_bridge
import cv2

STOP = 0
FORWARD = 1
BACKWARD = 2
TURN_LEFT = 3
TURN_RIGHT = 4
FOLLOW_LINE = 5
RED_LIGHT = 6

class Visualization():
    def __init__(self):
        
    # ---------------------     SUSCRIBERS       -----------------------------

        rospy.Subscriber("/line_detected", Bool, self.line_det_cb)
        rospy.Subscriber("/start", Bool, self.start_cb)
        rospy.Subscriber("/line", Int32, self.line_cb)
        rospy.Subscriber("/detected_color", String, self.color_cb)
        rospy.Subscriber("/sign", String, self.sign_cb)
        rospy.Subscriber("/camera/image_raw", Image, self.img_cb)

    # ---------------------     CONSTANTS       ------------------------------

#                                   #                                                            

            #image

#                                   #                                   
# Cropped image  #  Robot data:
#                #   Current State: 
#                #   Last Signal:
#                #   Traffic Light:
#                #   Line state:    Idx:
#                #   Linear:        Angular:   

        self.line_idx = 48
        self.line_detected = False
        self.color = None
        self.sign = "None"
        self.start_robot = False
        self.current_state = None
        self.linear_vel = 0
        self.angular_vel = 0
        self.image = None  
        self.crop = None
        self.bridge = cv_bridge.CvBridge()  # cv_bridge

        self.freq = 20
        self.rate = rospy.Rate(self.freq)  
        self.Dt = 1/float(self.freq)

    #####

        # white = [255,255,255]    
        # img = np.ones((180,320,1))        

        # crop = np.random.random((30, 30))
        # crop = np.resize(crop,(30,30,1))
        # title_crop = np.zeros((10,30,1))
        # crop_w_title = np.concatenate((title_crop,crop), axis=0)

        # data = np.zeros((30,290,1))
        # title_data = np.random.random((10, 290,1))
        # data_w_title = np.concatenate((title_data,data), axis=0)

        # down_image = np.concatenate((crop_w_title,
        #                              data_w_title), axis=1)

        # self.full_image = np.concatenate((img, down_image), axis=0)
        #self.full_image = cv2.copyMakeBorder(self.full_image,3,3,3,3,
        #                                    cv2.BORDER_CONSTANT,value= white)

        self.window = cv2.namedWindow("Donatello Visualizer",cv2.WINDOW_NORMAL)
        

    ####


    # ---------------------     CALLBACKS       -------------------------------
    
    def start_cb(self, msg):
        self.start_robot = msg.data

    def line_cb(self, msg):
        self.line_idx = msg.data

    def sign_cb(self, msg):
        self.sign = msg.data

    def color_cb(self, msg):
        self.color = msg.data

    def line_det_cb(self, msg):
        self.line_detected = msg

    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.image = cv2.resize(self.image,(320,180))
        self.crop = self.image[0:30, 0:30].copy()

    # ------------------- VISUALS ----------------------------------------

    def main(self):
        while not rospy.is_shutdown():
            if(self.image is not None and self.image.shape[1] == 320):
                im = self.image.copy()
                title_crop = np.zeros((10,30,3))
                crop_w_title = np.concatenate((title_crop,self.crop), axis=0)
                title_data = np.random.random((10, 290,3))
                data = np.zeros((30,290,3))
                data_w_title = np.concatenate((title_data,data), axis=0)
                
              
                down_image = np.concatenate((crop_w_title,
                                        data_w_title), axis=1)

                print(im.shape)
                print(down_image.shape)

                self.full_image = np.concatenate((self.image, down_image), axis=0)

                cv2.imshow("Donatello Visualizer",self.full_image)
                if cv2.waitKey(1) & 0xFF is ord('q'):
                    cv2.destroyAllWindows()
                    print("Stop programm and close all windows")
                    break
            self.rate.sleep()  


# ------------------- MAIN -----------------------------------------------

if __name__ == '__main__':
    rospy.init_node('Visualization_node', anonymous=True)
    rospy.loginfo("Node initialized")
    visuals = Visualization()
    visuals.main()


