#!/usr/bin/env python3
###############################################################################
# Includes
###############################################################################

import struct
from xml.sax.handler import property_xml_string
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from iii_interfaces.msg import Powerline, PowerlineDirection
#import ros2_numpy

import cv2 as cv
from cv_bridge import CvBridge

import numpy as np

import time
import copy
import sys
import os
import math 
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

from threading import Thread, Lock

from sshkeyboard import listen_keyboard


###############################################################################
#  Defines
###############################################################################
global image_width
image_width  = 640 #1920
global image_height
image_height  = 480 #1080
global img_dims
img_dims = (image_width, image_height)



###############################################################################
# Class
###############################################################################

class IdDrawer(Node):

    key_thread = None

    def __init__(self):
        self.pl_ = None
        self.pl_buff_ = []

        self.pl_dir_ = None
        self.pl_dir_buff_ = []

        self.buff_size_ = 2

        self.pl_lock_ = Lock()
        self.dir_lock_ = Lock()

        self.key_thread = Thread(target=self.keylistener, args=())

        self.pressed_keys = []
        self.typed_ID = -1

        super().__init__("image_drawer")
        self.pl_sub_ = self.create_subscription(
            Powerline,
            "/pl_mapper/powerline",	
            self.on_pl_msg,
            10
        )

        self.pl_direction_sub_ = self.create_subscription(
            PoseStamped,
            "/pl_dir_computer/powerline_direction",	
            self.on_pl_direction,
            10
        )

        self.drawn_img_pub_ = self.create_publisher(
            Image,
            "/ID_image",
            10
        )

        self.ID_pub_ = self.create_publisher(
            Int32,
            "/typed_ID",
            10
        )


        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.draw_image)


    def on_pl_msg(self, msg):
        if self.pl_lock_.acquire(blocking=True):
            self.pl_ = msg
            self.pl_buff_.append(self.pl_)
            if len(self.pl_buff_) == self.buff_size_+1:
                self.pl_buff_.pop(0)
            self.pl_lock_.release()


    def on_pl_direction(self, msg):
        if self.dir_lock_.acquire(blocking=True):
            self.pl_dir_ = msg
            self.pl_dir_buff_.append(self.pl_dir_)
            if len(self.pl_dir_buff_) == self.buff_size_+1:
                self.pl_dir_buff_.pop(0)
            self.dir_lock_.release()


    def rotate_xy(self, x, y, angle):

        rot_mat = np.matrix([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])

        xy = np.matrix([[x],[y]])

        rot_xy = np.matmul(rot_mat, xy)

        return rot_xy[0][0], rot_xy[1][0]


    def draw_image(self):

        if len(self.pl_buff_) < self.buff_size_-1 or len(self.pl_dir_buff_) < self.buff_size_-1:
            return

        pl = []
        if self.pl_lock_.acquire(blocking=True):
            if len(self.pl_buff_) == self.buff_size_-1:
                pl_temp = self.pl_
                for i in range(pl_temp.count):
                    pl.append([pl_temp.poses[i].position.x, pl_temp.poses[i].position.y, pl_temp.poses[i].position.z])
                pl = np.asarray(pl)
            else:
                pl_temp = self.pl_buff_.pop(0)
                for i in range(pl_temp.count):
                    pl.append([pl_temp.poses[i].position.x, pl_temp.poses[i].position.y, pl_temp.poses[i].position.z])
                pl = np.asarray(pl)
            self.pl_lock_.release()

        if self.dir_lock_.acquire(blocking=True):
            temp_dir = self.pl_dir_buff_.pop(0)
            
            pl_dir = math.atan2(
                2*(temp_dir.pose.orientation.w*temp_dir.pose.orientation.z+temp_dir.pose.orientation.x*temp_dir.pose.orientation.y),
                1-2*(temp_dir.pose.orientation.y*temp_dir.pose.orientation.y+temp_dir.pose.orientation.z*temp_dir.pose.orientation.z)
            )
            
            self.dir_lock_.release()


        # something wrong with rotation ?
        
        rotated_points_x = []
        points_z = []

        p_x = []
        p_y = []
        
        for i in range(len(pl)):
            to_rot_x = pl[i][0]
            to_rot_y = pl[i][1]
            # print("X,Y: ", to_rot_x, to_rot_y)
            rot_x, rot_y = self.rotate_xy(to_rot_x, to_rot_y, -pl_dir+1.57)
            # print("rot X,Y: ", rot_x, rot_y)
            rotated_points_x.append(rot_x)
            points_z.append(pl[i][2]) 

            p_x.append(pl[i][0])
            p_y.append(pl[i][2])


        fig, ax = plt.subplots()
        ax.scatter(rotated_points_x, points_z, linewidth=0.000001, color='green', label='Powerlines (#ID)')
        ax.scatter(0, 0, linewidth=0.000001, color='red', label='Ego', marker='X')

        n = []

        for i in range(pl_temp.count):
            n.append(pl_temp.ids[i])

        for i, txt in enumerate(n):
            ax.annotate(txt, (rotated_points_x[i], points_z[i]))

        ax.legend()

        plt.axis('square')

        # plt.show()

        fig.canvas.draw()

        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        # img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        # img is rgb, convert to opencv's default bgr
        img = cv.cvtColor(img,cv.COLOR_RGB2BGR)

        cvb = CvBridge()
        msg = cvb.cv2_to_imgmsg(img, encoding="bgr8")

        self.drawn_img_pub_.publish(msg)

        plt.cla()
        plt.clf()
        plt.close('all')



    ## Key press handling

    def press(self, key):
        numbers = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
        
        if key == 'e' or key == 'enter':

            if len(self.pressed_keys) < 1:
                print("Key sequence has length 0, no ID submitted")

            else:
                key_string = ""

                for i in range(len(self.pressed_keys)):
                    key_string = key_string + str(self.pressed_keys[i])
                    self.typed_ID = int(key_string)

                if len(self.pressed_keys) > 10 or self.typed_ID > 2147483647:
                    print("Entered ID outside range [0, 2147483648], clearing sequence")
                    self.pressed_keys.clear()
                    print("Type ID - current key sequence: [ ]")
                    print("Press 'del' to clear or 'enter' to submit sequence\n")
                    return

                print("ID", self.typed_ID, "submitted, clearing sequence")
                self.pressed_keys.clear()

                msg = Int32()
                msg.data = self.typed_ID
                self.ID_pub_.publish(msg)
        
        elif key == 'x' or key == 'delete':
                print("Clearing typed sequence")
                self.pressed_keys.clear()

        elif key == 'backspace':
            print("Deleted latest entry")

            if len(self.pressed_keys) > 0:
                self.pressed_keys.pop() 

        elif key in numbers:
            self.pressed_keys.append(key)

        else:
            print("Unsupported command:", key)

        key_string = ""
        for i in range(len(self.pressed_keys)):
            key_string = key_string + str(self.pressed_keys[i])
        print("Type ID - current key sequence: [", key_string, "]")
        print("Press 'del' to clear or 'enter' to submit sequence\n")
        

    def keylistener(self):

        print("\nEnter powerline ID")
        print("Press 'del' to clear or 'enter' to submit sequence\n")

        listen_keyboard(
            on_press=self.press,
        )





###############################################################################
# Main
###############################################################################


if __name__ == "__main__":
    rclpy.init()

    print("Starting IdDrawer node")

    minimal_publisher = IdDrawer()

    minimal_publisher.key_thread.start()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()