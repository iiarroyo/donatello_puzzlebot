#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from imutils.perspective import four_point_transform
import matplotlib.pyplot as plt
from tensorflow.keras.models import load_model
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Import model
model = load_model(r"/home/mikelv21/puzzlebot_ws/src/twist_example/scripts/traffic_classifier.h5")

signs = {14:"Stop",
         32:"NoLimitVel",
         33:"TurnRight",
         34:"TurnLeft",
         35:"GoAhead"}

# Define range HSV for blue color of the traffic sign
lower_blue = np.array([90,80,50])
upper_blue = np.array([110,255,255])
lower_red = np.array([0,50,50]) #example value
upper_red = np.array([10,255,255]) #example value
MinRadius=10
MaxRadius=21
Param1=55
Param2=18

def empty(x):
    pass

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters",640,240)
cv2.createTrackbar("MinRadius","Parameters",1,255,empty)
cv2.createTrackbar("MaxRadius","Parameters",1,255,empty)
cv2.createTrackbar("Param1","Parameters",10,200,empty)
cv2.createTrackbar("Param2","Parameters",1,100,empty)
cv2.setTrackbarPos("MinRadius","Parameters",MinRadius)
cv2.setTrackbarPos("MaxRadius","Parameters",MaxRadius)
cv2.setTrackbarPos("Param1","Parameters",Param1)
cv2.setTrackbarPos("Param2","Parameters",Param2)

cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
cv2.namedWindow("Cropped", cv2.WINDOW_NORMAL)

def findTrafficSign(image,text_s,text_c):
    text_2 = "Last signal: " + text_c
    text = text_s
    MinRadius= cv2.getTrackbarPos("MinRadius","Parameters")
    MaxRadius= cv2.getTrackbarPos("MaxRadius","Parameters")
    Param1= cv2.getTrackbarPos("Param1","Parameters")
    Param2= cv2.getTrackbarPos("Param2","Parameters")

    box = None
    circles = None
    '''
    This function find blobs with blue color on the image.
    After blobs were found it detects the largest square blob, that must be the sign.
    '''
    frame = image.copy()
    #frame = cv2.imread(image_path)
    #frameArea = frame.shape[0]*frame.shape[1]
    
    # convert color image to HSV color scheme
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)    
    imgGray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    imgGray=cv2.medianBlur(imgGray,5)
    rows=imgGray.shape[0]

    # define kernel for smoothing   
    kernel = np.ones((3,3),np.uint8)
    # extract binary image with active blue regions

    # morphological operations
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # find contours in the mask
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts_red = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # define variables to hold values during loop
    largestArea = 0
    largestRect = None
    min_area = 750
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
                largestRect = box
    
    if len(cnts_red)>0:
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
                largestRect = box

    if area < min_area:
        circles=cv2.HoughCircles(imgGray,cv2.HOUGH_GRADIENT,1,rows/8,param1=Param1,
                                    param2=Param2,minRadius=MinRadius, maxRadius=MaxRadius)  
        if(circles is not None):
            circles=np.uint16(np.around(circles))
            for i in circles[0,:]:
                # dibujar circulo 
                cv2.circle(frame, (i[0], i[1]), i[2], (0,0,255), 2)
                x_i = i[0] - i[2] 
                x_f = i[0] + i[2] 
                y_i = i[1] - i[2]  
                y_f = i[1] + i[2] 

                box = [[0,0],[x_i,y_i],[0,0],[x_f,y_f]]

    if area>min_area:
        cv2.drawContours(frame,[largestRect],0,(0,0,255),2)
    
    # show original image
    cv2.putText(frame, text,(10, 20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
    cv2.putText(frame, text_2,(10, 150),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
    cv2.imshow("Original", frame)   
    #print(area)
    if  area>min_area or circles is not None:
        return box
    else:
        return None

class Observer():
    def __init__(self):
        
        rospy.Subscriber("/video_source/raw", Image, self.img_cb)
        self.pred_pub = rospy.Publisher("sign", String, queue_size=1)
        
        self.image = np.zeros((0, 0))  
        self.p_img = np.zeros((0, 0))  
        self.bridge = cv_bridge.CvBridge()
        last_signal = "No signal detected"
        last_correct = "Start"
        frec = 10  
        r = rospy.Rate(frec)  # Hz
        print("Node initialized {0}Hz".format(frec))
        while not rospy.is_shutdown():
            if self.image is not None and self.image.size > 0:
                image_copy = self.image
                coords = findTrafficSign(image_copy,last_signal,last_correct)
                #print(coords)
                # To add the cropped image a tolerance
                t = 10
        
                rgb_image = cv2.cvtColor(image_copy, cv2.COLOR_BGR2RGB)
                if  (coords is not None):
                    xi = coords[1][0]
                    yi = coords[1][1]
                    xf = coords[3][0]
                    yf = coords[3][1]
                    cropped_im = rgb_image[yi-t:yf+t, xi-t:xf+t].copy()
                    if(cropped_im.shape[0]>0 and cropped_im.shape[1]>0):
                        cv2.imshow("Cropped", cropped_im)
                        cropped_im = cv2.resize(cropped_im,(30,30))
                        pred = np.argmax(model.predict(cropped_im.reshape(1,30,30,3)))
                        if(pred in [14,32,33,34,35]):
                            #print("Signal predicted: ",signs[pred])
                            last_signal = "Signal predicted: " + str(signs[pred])
                            last_correct = str(signs[pred])
                        else:
                            #print("Unknow signal")
                            last_signal = "Unknow signal"
                    else:
                        #print("Not signal detected")
                        last_signal = "No image"
                else:
                    #print("Not signal detected")
                    last_signal = "No signal detected"
                
                self.pred_pub.publish(last_correct)

                if cv2.waitKey(1) & 0xFF is ord('q'):
                    cv2.destroyAllWindows()
                    print("Stop programm and close all windows")
                    break

            r.sleep()

    def img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

if __name__ == "__main__":
    rospy.init_node("img_detector", anonymous=True)
    Observer()