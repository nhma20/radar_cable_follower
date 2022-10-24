#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np


global image_width
image_width  = 640
global image_height
image_height  = 480
global dims
dims = (image_width, image_height)


class ImageUndistorter(Node):
	def __init__(self):
	
		print("Running image_undistorter node")
		
		super().__init__("image_undistorter")
		
		self.camera_sub_ = self.create_subscription(
			CompressedImage,
			"/image_raw/compressed",	
			self.on_img_msg,
			10
		)

		self.undistorted_img_pub_ = self.create_publisher(
			Image,
			"/image_raw",
			10
		)
		
		self.mtx = np.asarray([[730.47250848,   0.,         380.03646788],
			 [  0.,         740.25842329, 306.83207802],
			 [  0.,           0.,           1.        ]])

		self.dist = np.asarray([[-4.69333244e-01,  3.91452924e-01,  5.37165492e-04,  3.47175268e-04,
				  -3.84039514e-01]])

		self.newcameramtx = np.asarray([[685.56347656,   0.,         404.85563285],
			 [  0.,         739.02459717, 306.3206725 ],
			 [  0.,           0.,           1.        ]])


	def on_img_msg(self, msg):
		#print("Received image")	
		
		np_arr = np.asarray(msg.data, dtype=np.uint8)
		cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		cvb = CvBridge()

		dst = cv2.undistort(cv_img, self.mtx, self.dist, None, self.newcameramtx)
		corrected = cv2.resize(dst, dims, interpolation = cv2.INTER_AREA)

		msg = cvb.cv2_to_imgmsg(corrected, encoding="bgr8")

		self.undistorted_img_pub_.publish(msg)
		#print("Undistorted image")
		
		
###############################################################################
# Main
###############################################################################

if __name__ == "__main__":
	rclpy.init()

	img_undistort = ImageUndistorter()

	rclpy.spin(img_undistort)

	img_undistort.destroy_node()
	rclpy.shutdown()
