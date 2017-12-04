#!/usr/bin/env python
import rospy
import numpy as np
import math
import argparse
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import cv2
import sys
import time
import threading
import argparse
import imutils
import message_filters

class depth_detect_cir(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.thread_lock = threading.Lock()
        self.active = True
        self.bridge = CvBridge()
        # set the range area for figure we detect
        self.MAXAREA = 30000
        self.MINAREA = 150
        self.ratio = 0.0
        # publish the image after detect
        self.pub_image_detect = rospy.Publisher("~image_with_detect", Image, queue_size=1)
        self.pub_image_HSV = rospy.Publisher("~image_with_HSV", Image, queue_size=1)
        #subscribe compressed image from camera
        self.cam_info = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo, timeout=None) 
        self.fx = self.cam_info.P[0]
        self.fy = self.cam_info.P[5]
        self.cx = self.cam_info.P[2]
        self.cy = self.cam_info.P[6]
        self.cv_depthimage2 = None
        # Bridge to convert ROS Image type to OpenCV Image type
        self.cv_bridge = CvBridge() 

        self.image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth_registered/sw_registered/image_rect_raw", Image)
        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 0.5)
        ts.registerCallback(self.rosRGBDCallBack)

        rospy.on_shutdown(self.custom_shutdown)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)

    def rosRGBDCallBack(self, rgb_data, depth_data):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            cv_depthimage = self.cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
            self.cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
        except CvBridgeError as e:
            print(e)
        #print "size of depth image = ", self.cv_depthimage2.shape
        #print "size of cv_image = ", cv_image.shape
        #print "size of dddd = ", cv_depthimage.shape
        self.cbFiguredetect(cv_image)
        #contours, mask_image = HSVObjectDetection(cv_image, toPrint = False)
        #print 'depth', cv_depthimage[240][320] 
        '''for cnt in contours:
            xp,yp,w,h = cv2.boundingRect(cnt)
            cv_depthimage2[int(yp)][int(xp)]/=1000.0
            # Get depth value from depth image, need to make sure the value is in the normal range 0.1-10 meter
            if not math.isnan(cv_depthimage2[int(yp)][int(xp)]) and cv_depthimage2[int(yp)][int(xp)] > 0.1 and cv_depthimage2[int(yp)][int(xp)] < 10.0:
                zc = cv_depthimage2[int(yp)][int(xp)]
                print 'zc', zc
            else:
                continue
                
            centerx, centery = xp+w/2, yp+h/2
            cv2.rectangle(cv_image,(xp,yp),(xp+w,yp+h),[0,255,255],2)
            
            showPyramid(centerx, centery, zc, w, h)
        '''
        #cv2.imshow("Contours", cv_depthimage)
        #cv2.waitKey(13)
    def getXYZ(self, xp, yp, zc, fx, fy, cx, cy):
        transform_matrix = np.mat([[fx, 0, cx],[0, fx, cy],[0, 0, 1]])
        img_p = np.mat([xp, yp, 1])
        inverse_transform_matrix = np.linalg.inv(transform_matrix)
        xd = inverse_transform_matrix * (img_p.transpose())
        xn = xd[0,0]  
        yn = xd[1,0]
        xc = xn*zc
        yc = yn*zc 
        return (xc,yc,zc)

    def check_square(self,box):
        dis1 = math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
        dis2 = math.sqrt((box[1][0] - box[2][0])**2 + (box[1][1] - box[2][1])**2)
        for i in box:
            if i[0] <= 10 or i[0] >= 620 or i[1] <= 10 or i[1] >= 470:
                return False
        if 1.1 >= (dis1/dis2) >= 0.9:
            return True
        return False
    # above function is used for getting rid of the unwanted square, like vertices of square which is out of the image

    def cbFiguredetect(self, image):
        #decode the image to cv_image which we used
        #narr = np.fromstring(image_msg.data, np.uint8)
        #image = cv2.imdecode(narr, cv2.IMREAD_COLOR)
        a=[0,0]
        box = [a,a,a,a]
        xp, yp = 0, 0
        # resize image
        resized = imutils.resize(image, width=600)
        hsv = cv2.cvtColor(resized,cv2.COLOR_BGR2HSV)
        lowerblack = np.array([0,0,0])
        upperblack = np.array([180,255,50])
        mask = cv2.inRange(hsv,lowerblack,upperblack)
        #mask = cv2.Canny(mask2,100,200)

        # Blurs an image using a Gaussian filter
        blurred = cv2.GaussianBlur(mask, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        self.ratio = image.shape[0] / float(resized.shape[0])   #find resized rate
        # use findContours to find the contour of figure
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_CCOMP  ,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[1]
        # initial ShapeDetector class
        sd = ShapeDetector()
        # find for each figure
        for c in cnts:      
            area = cv2.contourArea(c)                   
            if self.MAXAREA >= area >= self.MINAREA:    #check area is inrange or not
                shape = sd.detect(c)                        #check which shape is by using contours
                if shape is "circle":
                    M = cv2.moments(c)
                    if M["m00"] == 0 :
                        break
                    #find the center of figure
                    yp = int((M["m10"] / M["m00"]) * self.ratio)
                    xp = int((M["m01"] / M["m00"]) * self.ratio)
                    c = c.astype("float")
                    c *= self.ratio
                    c = c.astype("int")         #cast c to integer
                    rect = cv2.minAreaRect(c)   #draw the rectangle by contours
                    box = cv2.boxPoints(rect)   #find the vertices of rectangle
                    box = np.int0(box)          #transform float to integer
                    if self.check_square(box):
                        cv2.drawContours(image,[box], 0, (255, 0, 0), 2)  #draw the contours of outer square
                        cv2.putText(image, shape, (yp, xp), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 2) # print the shape of detail
                        zc = self.cv_depthimage2[int(xp)][int(yp)]/1000.0 #change unit to "meter"
                        #print zc
                        if zc == 0:
                            zc = 1
                        center = self.getXYZ(xp, yp, zc, self.fx, self.fy, self.cx, self.cy)
                        print "(x, y, z) = ", center
        image_msg_out = self.bridge.cv2_to_imgmsg(image, "bgr8")    
        image_msg_out_hsv = self.bridge.cv2_to_imgmsg(mask)         
        #image_msg_out.header.stamp = image.header.stamp


        self.pub_image_detect.publish(image_msg_out)
        self.pub_image_HSV.publish(image_msg_out_hsv)               #output inranged image
        
        #print xp, yp
        #return xp, yp

class ShapeDetector:
    def __init__(self):
        pass
 
    def detect(self, c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True) # calculates a contour curve length
        c1 = cv2.convexHull(c)
        approx = cv2.approxPolyDP(c1, 0.03 * peri, True) #Approximates a polygonal curve with curve length (define how curve would be a line)

        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"
 
        # if the shape has 4 vertices, it is either a square or a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            # a square will have an aspect ratio that is approximately equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.7 and ar <= 1.3 else "rectangle"
            # if it's not as us expect, generalize it as unidentified
            if h < 5 or w < 5:
                shape = "unidentified" 

        # otherwise, we assume the shape is a circle
        
        else:
            shape = "circle"
 
        # return the name of the shape
        return shape

if __name__ == "__main__":
    rospy.init_node("depth_detect_cir_node",anonymous=False)
    depth_detect_cir_node = depth_detect_cir()
    rospy.spin()