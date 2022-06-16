#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


# Define range HSV for blue color of the traffic sign
lower_blue = np.array([90, 80, 50])
upper_blue = np.array([110, 255, 255])
lower_red = np.array([0, 50, 50])  # example value
upper_red = np.array([10, 255, 255])  # example value
MinRadius = 11
MaxRadius = 20
Param1 = 34
Param2 = 22
min_area = 207

def empty(x):
   pass
# cv2.namedWindow("Parameters")
# cv2.resizeWindow("Parameters",640,240)
# cv2.createTrackbar("MinRadius","Parameters",1,255,empty)
# cv2.createTrackbar("MaxRadius","Parameters",1,255,empty)
# cv2.createTrackbar("Param1","Parameters",10,200,empty)
# cv2.createTrackbar("Param2","Parameters",1,100,empty)
# cv2.createTrackbar("Min_area","Parameters",1,2000,empty)

# cv2.setTrackbarPos("MinRadius","Parameters",MinRadius)
# cv2.setTrackbarPos("MaxRadius","Parameters",MaxRadius)
# cv2.setTrackbarPos("Param1","Parameters",Param1)
# cv2.setTrackbarPos("Param2","Parameters",Param2)
# cv2.setTrackbarPos("Min_area","Parameters",min_area)

#cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
#cv2.namedWindow("Cropped", cv2.WINDOW_NORMAL)


def findTrafficSign(image):
    # MinRadius= cv2.getTrackbarPos("MinRadius","Parameters")
    # MaxRadius= cv2.getTrackbarPos("MaxRadius","Parameters")
    # Param1= cv2.getTrackbarPos("Param1","Parameters")
    # Param2= cv2.getTrackbarPos("Param2","Parameters")
    # min_area = cv2.getTrackbarPos("Min_area","Parameters")
    # cv2.waitKey(1)
    box = None
    circles = None
    frame = image.copy()

    #frame = cv2.imread(image_path)
    #frameArea = frame.shape[0]*frame.shape[1]

    # convert color image to HSV color scheme
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define kernel for smoothing
    kernel = np.ones((3, 3), np.uint8)
    # extract binary image with active blue regions

    # morphological operations
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # find contours in the mask
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts_red = cv2.findContours(
        mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # define variables to hold values during loop
    largestArea = 0
    area = 0
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        for cnt in cnts:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # count euclidian distance for each side of the rectangle
            sideOne = np.linalg.norm(box[0]-box[1])
            sideTwo = np.linalg.norm(box[0]-box[3])
            # count area of the rectangle
            area = sideOne*sideTwo
            # find the largest rectangle within all contours
            if area > largestArea:
                largestArea = area

    if len(cnts_red) > 0:
        for cnt in cnts_red:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # count euclidian distance for each side of the rectangle
            sideOne = np.linalg.norm(box[0]-box[1])
            sideTwo = np.linalg.norm(box[0]-box[3])
            # count area of the rectangle
            area = sideOne*sideTwo
            # find the largest rectangle within all contours
            if area > largestArea:
                largestArea = area

    if area < min_area:
        imgGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        imgGray = cv2.medianBlur(imgGray, 5)
        rows = imgGray.shape[0]
        circles = cv2.HoughCircles(imgGray, cv2.HOUGH_GRADIENT, 1, rows/8, param1=Param1,
                                   param2=Param2, minRadius=MinRadius, maxRadius=MaxRadius)
        if(circles is not None):
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # dibujar circulo
                x_i = i[0] - i[2]
                x_f = i[0] + i[2]
                y_i = i[1] - i[2]
                y_f = i[1] + i[2]
                box = [[0, 0], [x_i, y_i], [0, 0], [x_f, y_f]]

    if area > min_area or circles is not None:
        return box
    else:
        return None


class CropJetson():
    def __init__(self):

        rospy.Subscriber("/video_source/raw", Image, self.img_cb)
        self.crop_pub = rospy.Publisher("cropped_image", Image, queue_size=10)
        self.pred_pub = rospy.Publisher("sign", String, queue_size=10)

        self.image = np.zeros((0, 0))
        self.cropped_img = np.zeros((0, 0))
        self.bridge = cv_bridge.CvBridge()
        self.img_back = Image()

        frec = 15
        r = rospy.Rate(frec)  # Hz
        print("Node initialized {0}Hz".format(frec))
        while not rospy.is_shutdown():
            if self.image is not None and self.image.size > 0:
                image_copy = self.image.copy()
                coords = findTrafficSign(image_copy)
                # print(coords)
                # To add the cropped image a tolerance
                t = 13
                rgb_image = cv2.cvtColor(image_copy, cv2.COLOR_BGR2RGB)
                if (coords is not None):
                    xi = coords[1][0]
                    yi = coords[1][1]
                    xf = coords[3][0]
                    yf = coords[3][1]
                    cropped_im = rgb_image[yi-t:yf+t, xi-t:xf+t].copy()
                    print(cropped_im.shape)
                    if(cropped_im.shape[0] > 30 and cropped_im.shape[1] > 30):
                        cropped_im = cv2.resize(cropped_im, (30, 30))
                        self.img_back = self.bridge.cv2_to_imgmsg(
                            cropped_im, encoding="passthrough")
                        self.crop_pub.publish(self.img_back)
                    # else:
                    #     self.pred_pub.publish("NONE")

            r.sleep()

    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


if __name__ == "__main__":
    rospy.init_node("img_cropper", anonymous=True)
    CropJetson()
