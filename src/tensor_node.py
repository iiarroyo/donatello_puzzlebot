#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class Prediction():
    def __init__(self):
        # Import model

        self.signs = {14: "Stop",
         32: "NoLimitVel",
         33: "TurnRight",
         34: "TurnLeft",
         35: "GoAhead"}

        self.model = load_model(r"/home/iiarroyo/catkin_ws/src/donatello_puzzlebot/traffic_classifier.h5")

        rospy.Subscriber("cropped_image", Image, self.img_cb)
        self.pred_pub = rospy.Publisher("sign", String, queue_size=10)

        self.image = np.zeros((0, 0))
        self.bridge = cv_bridge.CvBridge()
        self.last_correct = "Waiting"
        frec = 30
        r = rospy.Rate(frec)  # Hz

        print("Node initialized {0}Hz".format(frec))
        while not rospy.is_shutdown():
            image_copy = self.image.copy()
            if image_copy is not None and image_copy.size > 0:
                if(image_copy.shape[0] >= 30 and image_copy.shape[1] >= 30):
                    resized = cv2.resize(image_copy, (30, 30))
                    pred = np.argmax(self.model.predict(resized.reshape(1, 30, 30, 3)))
                    if(pred in [14, 32, 33, 34, 35]):
                        self.last_correct = str(self.signs[pred])

                        print(self.last_correct)
                        self.pred_pub.publish(self.last_correct)
            r.sleep()

    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


if __name__ == "__main__":
    rospy.init_node("Predictor", anonymous=True)
    Prediction()
