#!/usr/bin/env python
import rospy
import numpy as np
import math
import argparse
from duckietown_msgs.msg import  Twist2DStamped
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import cv2
import sys
import time
import threading
import argparse
import imutils

class black_cir_detect(object):
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
		self.sub_image_origin = rospy.Subscriber("arg4/camera_node/image/compressed", CompressedImage, self.cbImage, queue_size=10)
		rospy.on_shutdown(self.custom_shutdown)
		rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

	def custom_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %self.node_name)
		rospy.sleep(0.5) #To make sure that it gets published.
		rospy.loginfo("[%s] Shutdown" %self.node_name)

	def cbImage(self, image_msg):
		if not self.active:
			return
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()

	def processImage(self, image_msg):
		if not self.thread_lock.acquire(False):
			return
		try:
			self.cbFiguredetect(image_msg)
		finally:
			self.thread_lock.release()
	
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

	def cbFiguredetect(self, image_msg):
		#decode the image to cv_image which we used
		narr = np.fromstring(image_msg.data, np.uint8)
		image = cv2.imdecode(narr, cv2.IMREAD_COLOR)

		a=[0,0]
		box = [a,a,a,a]

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
 		self.ratio = image.shape[0] / float(resized.shape[0])	#find resized rate
 		# use findContours to find the contour of figure
		cnts = cv2.findContours(thresh.copy(), cv2.RETR_CCOMP  ,cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[1]
		# initial ShapeDetector class
		sd = ShapeDetector()
		# find for each figure
		for c in cnts:		
			area = cv2.contourArea(c) 					
			if self.MAXAREA >= area >= self.MINAREA:	#check area is inrange or not
				shape = sd.detect(c)						#check which shape is by using contours
				if shape is "circle":
					M = cv2.moments(c)
					if M["m00"] == 0 :
						break
					#find the center of figure
					cX = int((M["m10"] / M["m00"]) * self.ratio)
					cY = int((M["m01"] / M["m00"]) * self.ratio)
					c = c.astype("float")
					c *= self.ratio
					c = c.astype("int")			#cast c to integer
					rect = cv2.minAreaRect(c)	#draw the rectangle by contours
					box = cv2.boxPoints(rect)   #find the vertices of rectangle
					box = np.int0(box)          #transform float to integer
					if self.check_square(box):
						cv2.drawContours(image,[box], 0, (255, 0, 0), 2)  #draw the contours of outer square
						cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 2) # print the shape of detail

		image_msg_out = self.bridge.cv2_to_imgmsg(image, "bgr8") 	
		image_msg_out_hsv = self.bridge.cv2_to_imgmsg(mask)			
		image_msg_out.header.stamp = image_msg.header.stamp


		self.pub_image_detect.publish(image_msg_out)
		self.pub_image_HSV.publish(image_msg_out_hsv) 				#output inranged image
 
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
	rospy.init_node("black_circle_node",anonymous=False)
	black_circle_node = black_cir_detect()
	rospy.spin()