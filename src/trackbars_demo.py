import cv2

def empty(a):
    pass

cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Trackbars",640,300)
cv2.createTrackbar("Hue Min","Trackbars",0,255,empty)
cv2.createTrackbar("Hue Max","Trackbars",0,255,empty)
cv2.createTrackbar("Sat Min","Trackbars",0,255,empty)
cv2.createTrackbar("Sat Max","Trackbars",0,255,empty)
cv2.createTrackbar("Val Min","Trackbars",0,255,empty)
cv2.createTrackbar("Val Max","Trackbars",0,255,empty)

ch = None
while ch != 27:
    ch = cv2.waitKey(0)