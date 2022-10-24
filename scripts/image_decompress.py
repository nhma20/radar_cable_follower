#!/usr/bin/env python3

# inspired by: https://ncnynl.com/archives/201611/1067.html (for ROS1, in Chinese)

###############################################################################
# Includes
###############################################################################

import rclpy
import cv2 as cv
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class ImageDecompressor(Node):
    def __init__(self):
        super().__init__("image_decompressor")
        self.camera_sub_ = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",	
            self.on_img_msg,
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            "/decompressed_image",
            10
        )

        self.bridge = CvBridge()


    def on_img_msg(self, msg):
        print("Compressed image received")
        #np_arr = np.fromstring(msg.data, np.uint8)
        np_arr = np.asarray(msg.data, dtype=np.uint8)
        cv_img = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        #cv.imshow('cv_img', cv_img)
        #cv.waitKey(2)
        msg = self.bridge.cv2_to_imgmsg(cv_img,encoding="passthrough")
        msg.header.frame_id="drone"
        self.image_pub.publish(msg)

###############################################################################
# Main
###############################################################################

if __name__ == "__main__":
    rclpy.init()
    
    img_decomp = ImageDecompressor()
    
    rclpy.spin(img_decomp)
    
    img_decomp.destroy_node()
    rclpy.shutdown()
