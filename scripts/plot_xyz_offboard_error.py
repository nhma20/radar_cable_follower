#!/usr/bin/env python3
###############################################################################
# Includes
###############################################################################

import struct
from xml.sax.handler import property_xml_string
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseArray

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
from matplotlib.ticker import MaxNLocator

from threading import Thread, Lock

from sshkeyboard import listen_keyboard

import pandas as pd



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
       
        self.ego_lock_ = Lock()
        self.pl_lock_ = Lock()
        self.pair_lock = Lock()
        self.distance_lock = Lock()

        self.follow_pose = [None]
        self.ego_pose = [None]
        self.pose_pair_array = [] # ego, follow

        self.distance = 0

        self.distance_array = []
        self.horizontal_error_array = []
        self.vertical_error_array = []
        self.yaw_error_array = []

        self.vline_x_arr = []

        self.key_thread = Thread(target=self.keylistener, args=())

        super().__init__("image_drawer")

        self.pl_sub_ = self.create_subscription(
            PoseArray,
            "/vis_powerlines_array",	
            self.on_pose_msg,
            10
        )

        self.odo_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/vehicle_odometry/out",	
            self.on_odo_msg,
            10
        )




    def on_pose_msg(self, msg):
        if self.pl_lock_.acquire(blocking=True):
            try:
                x_tmp0 = msg.poses[0].position.x
                y_tmp0 = msg.poses[0].position.y
                z_tmp0 = msg.poses[0].position.z
                q0 = msg.poses[0].orientation.x
                q1 = msg.poses[0].orientation.y
                q2 = msg.poses[0].orientation.z
                q3 = msg.poses[0].orientation.w
                self.follow_pose = [x_tmp0, y_tmp0, z_tmp0, q0, q1, q2, q3]
    
            except:
                print("Probably empty PoseArray")
        self.pl_lock_.release() 


    def on_odo_msg(self, msg):
        if self.ego_lock_.acquire(blocking=True):

            x_tmp1 = msg.x
            y_tmp1 = -msg.y
            z_tmp1 = -msg.z
            q0 = msg.q[0]
            q1 = msg.q[1]
            q2 = msg.q[2]
            q3 = msg.q[3]
            self.ego_pose = [x_tmp1, y_tmp1, z_tmp1, q0, q1, q2, q3]

            if self.pl_lock_.acquire(blocking=True):
                # print("123")
                if self.pair_lock.acquire(blocking=True):
                    if self.follow_pose[0] is not None:
                        self.pose_pair_array.append([self.ego_pose, self.follow_pose])
                    self.pair_lock.release()
                self.pl_lock_.release()   

            self.ego_lock_.release()


    # def on_pose_msg(self, msg):
    #     if self.pl_lock_.acquire(blocking=True):
    #         try:
    #             x_tmp0 = msg.poses[0].position.x
    #             y_tmp0 = msg.poses[0].position.y
    #             z_tmp0 = msg.poses[0].position.z
    #             q0 = msg.poses[0].orientation.x
    #             q1 = msg.poses[0].orientation.y
    #             q2 = msg.poses[0].orientation.z
    #             q3 = msg.poses[0].orientation.w
    #             self.follow_pose = [x_tmp0, y_tmp0, z_tmp0, q0, q1, q2, q3]

    #             if self.ego_lock_.acquire(blocking=True):
    #                 print("123")
    #                 if self.pair_lock.acquire(blocking=True):
    #                     if self.ego_pose[0] is not None:
    #                         self.pose_pair_array.append([self.ego_pose, self.follow_pose])
    #                     self.pair_lock.release()
    #                 self.ego_lock_.release()  
    
    #         except:
    #             print("Probably empty PoseArray")
    #     self.pl_lock_.release() 


    # def on_odo_msg(self, msg):
    #     if self.ego_lock_.acquire(blocking=True):

    #         x_tmp1 = msg.x
    #         y_tmp1 = -msg.y
    #         z_tmp1 = -msg.z
    #         q0 = msg.q[0]
    #         q1 = msg.q[1]
    #         q2 = msg.q[2]
    #         q3 = msg.q[3]
    #         self.ego_pose = [x_tmp1, y_tmp1, z_tmp1, q0, q1, q2, q3] 

    #         self.ego_lock_.release()





    def draw_image(self):

        if self.ego_lock_.acquire(blocking=True):
            if self.pl_lock_.acquire(blocking=True):

                # calculate distance
                try:
                    distance = 0
                    for i in range(len(self.pose_pair_array)-1):

                        delta_x = self.pose_pair_array[i+1][0][0] - self.pose_pair_array[i][0][0]
                        delta_y = self.pose_pair_array[i+1][0][1] - self.pose_pair_array[i][0][1]
                        delta_z = self.pose_pair_array[i+1][0][2] - self.pose_pair_array[i][0][2]

                        # print("Deltas: ", delta_x, delta_y, delta_z)


                        delta_distance = math.sqrt( math.pow( delta_x , 2) + math.pow( delta_y , 2) + math.pow( delta_z , 2) )
                        distance = distance + delta_distance

                        self.distance_array.append(distance)

                    print("Distance covered: ", distance)
                
                except:
                    print("Try again")

                
                # calculate horizontal error
                try:
                    for i in range(len(self.pose_pair_array)-1):

                        delta_x = 0 #self.pose_pair_array[i][0][0] - self.pose_pair_array[i][1][0]
                        delta_y = self.pose_pair_array[i][0][1] - self.pose_pair_array[i][1][1]

                        # print("Deltas: ", delta_x, delta_y)

                        if delta_y < 0:
                            hor_error = math.sqrt( math.pow( delta_x , 2) + math.pow( delta_y , 2) )
                        else:
                            hor_error = - math.sqrt( math.pow( delta_x , 2) + math.pow( delta_y , 2) )

                        z = 10
                        tmp_sum = 0
                        if len(self.horizontal_error_array) > z:
                            for k in range(z-1):
                                tmp_sum = tmp_sum + self.horizontal_error_array[len(self.horizontal_error_array)-(k+1)]
                            self.horizontal_error_array.append( ( tmp_sum + hor_error ) / z)
                        else:
                            self.horizontal_error_array.append(hor_error)

                        # if abs(hor_error) < 0.03:
                        #     self.horizontal_error_array.append(self.horizontal_error_array[-1])
                        # else:
                        #     self.horizontal_error_array.append(hor_error)

                    print("Horizontal error: ", hor_error)
                    # print("hor error array: ", self.horizontal_error_array)
                
                except:
                    print("Try again")





                # calculate vertical error
                try:
                    if self.pair_lock.acquire(blocking=True):
                        for i in range(len(self.pose_pair_array)-1):

                            # print(self.pose_pair_array[i][0][2])
                            # print(self.pose_pair_array[i])
                            delta_z = abs(self.pose_pair_array[i][0][2] - self.pose_pair_array[i][1][2]) #- 5

                            # print("Deltas: ", delta_x, delta_y, delta_z)

                            self.vertical_error_array.append(delta_z)
                        self.pair_lock.release()

                except:
                    print("Try again")

                print("Vertical error: ", delta_z)
                
               



                # calculate yaw error
                def quatMultiply(quat1, quat2):

                    ret_quat = [
                        quat1[0] * quat2[3] + quat1[1] * quat2[2] - quat1[2] * quat2[1] + quat1[3] * quat2[0],
                        -quat1[0] * quat2[2] + quat1[1] * quat2[3] + quat1[2] * quat2[0] + quat1[3] * quat2[1],
                        quat1[0] * quat2[1] - quat1[1] * quat2[0] + quat1[2] * quat2[3] + quat1[3] * quat2[2],
                        -quat1[0] * quat2[0] - quat1[1] * quat2[1] - quat1[2] * quat2[2] + quat1[3] * quat2[3]
                    ]

                    return ret_quat


                def quatYaw(quat): 
                    siny_cosp = 2 * (quat[3] * quat[2] + quat[0] * quat[1])
                    cosy_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])
                    return math.atan2(siny_cosp, cosy_cosp)


                for i in range(len(self.pose_pair_array)-1):

                    # msg->q[3],
                    # msg->q[0],
                    # msg->q[1],
                    # msg->q[2]
                    quat1 = [self.pose_pair_array[i][0][6], self.pose_pair_array[i][0][3], self.pose_pair_array[i][0][4], self.pose_pair_array[i][0][5]]
                   
                    RollYaw_PI_quat = [0.0, -1.0, 0.0, 0.0]

                    quat1 = quatMultiply(quat1, RollYaw_PI_quat)

                    ego_yaw = quatYaw(quat1)

                    # quat2 = [self.pose_pair_array[i][1][6], self.pose_pair_array[i][1][3], self.pose_pair_array[i][1][4], self.pose_pair_array[i][1][5]]
                    quat2 = [self.pose_pair_array[i][1][3], self.pose_pair_array[i][1][4], self.pose_pair_array[i][1][5], self.pose_pair_array[i][1][6]]

                    pose_yaw = quatYaw(quat2)

                    yaw_error = ego_yaw - pose_yaw

                    self.yaw_error_array.append(yaw_error)

                print("Yaw error: ", yaw_error, "ego: ", ego_yaw, "pose: ", pose_yaw)

                   
                

                
        self.pl_lock_.release() 
        self.ego_lock_.release()

        print("Array lengths: ", len(self.distance_array), len(self.horizontal_error_array), len(self.vertical_error_array), len(self.yaw_error_array))


        DF = pd.DataFrame(self.distance_array)
        DF.to_csv("travelled_distance_array.csv")

        DF = pd.DataFrame(self.horizontal_error_array)
        DF.to_csv("horizontal_error_array.csv")

        DF = pd.DataFrame(self.vertical_error_array)
        DF.to_csv("vertical_error_array.csv")

        DF = pd.DataFrame(self.yaw_error_array)
        DF.to_csv("yaw_error_array.csv")


        ## Mutliplot

        fig, ((ax1, ax2, ax3)) = plt.subplots(nrows=3, ncols=1, sharex=True, sharey=False)

        xs = range(len(self.distance_array))
        labels = self.distance_array

        def format_fn(tick_val, tick_pos):
            if int(tick_val) in xs:
                return int(labels[int(tick_val)])
            else:
                return ''

        ax1.plot(self.horizontal_error_array, color='red')
        ax1.hlines(y=0, xmin=0, xmax=len(self.distance_array), colors='gray', linestyles='--', lw=1)
        print(len(self.distance_array))
        print(min(self.distance_array))
        print(max(self.distance_array))
        for i in range(len(self.vline_x_arr)):
            print(i)
            print(self.distance_array[self.vline_x_arr[i]])
            # ax1.vlines(self.distance_array[self.vline_x_arr[i]], min(self.horizontal_error_array), max(self.horizontal_error_array), linestyles ="dotted", colors ="black")
            ax1.vlines(self.vline_x_arr[i], min(self.horizontal_error_array), max(self.horizontal_error_array), linestyles ="dotted", colors ="black")

        ax2.plot(self.vertical_error_array, color='green')
        ax2.hlines(y=0, xmin=0, xmax=len(self.distance_array), colors='gray', linestyles='--', lw=1)
        for i in range(len(self.vline_x_arr)):
            ax2.vlines(self.vline_x_arr[i], min(self.vertical_error_array), max(self.vertical_error_array), linestyles ="dotted", colors ="black")


        ax3.xaxis.set_major_formatter(format_fn)
        ax3.xaxis.set_major_locator(MaxNLocator(integer=True))
        # ax3.plot(self.distance_array, self.distance_array, color='blue', label='yaw')
        ax3.plot(self.yaw_error_array, color='blue')
        ax3.hlines(y=0, xmin=0, xmax=len(self.distance_array), colors='gray', linestyles='--', lw=1)
        # for i in range(len(self.vline_x_arr)):
        #     ax3.vlines(self.vline_x_arr[i], min(self.yaw_error_array), max(self.yaw_error_array), linestyles ="dotted", colors ="black", label='tower crossing')
        if len(self.vline_x_arr) > 0:
            ax3.vlines(self.vline_x_arr[0], min(self.yaw_error_array), max(self.yaw_error_array), linestyles ="dotted", colors ="black")
            ax3.vlines(self.vline_x_arr[1], min(self.yaw_error_array), max(self.yaw_error_array), linestyles ="dotted", colors ="black", label='tower')

        ax1.set_ylabel('Horizontal (m)')
        ax2.set_ylabel('Vertical (m)')
        ax3.set_ylabel('Yaw (rad)')

        ax3.set_xlabel('Followed distance (m)')

        # fig.legend()

        # k = 10
        # default_x_ticks = np.arange(k)

        # custom_x_ticks = []

        # vals_to_skip = len(self.distance_array) / (k-1)

        # custom_x_ticks.append(0)
        # for i in range(k-1):
        #     custom_x_ticks.append( int( self.distance_array[ math.floor( (i+1) * vals_to_skip ) -1] ) )

        # plt.xticks(default_x_ticks, custom_x_ticks)


        ## Single plot

        # fig, ((ax1)) = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=False)

        # xs = range(len(self.distance_array))
        # labels = self.distance_array

        # def format_fn(tick_val, tick_pos):
        #     if int(tick_val) in xs:
        #         return int(labels[int(tick_val)])
        #     else:
        #         return ''

        # ax1.plot(self.horizontal_error_array, color='red', label='horizontal', linewidth=0.7, alpha=0.7)
        # ax1.plot(self.vertical_error_array, color='green', label='vertical', linewidth=0.7, alpha=0.7)
        # ax1.plot(self.yaw_error_array, color='blue', label='yaw', linewidth=0.7, alpha=0.7)
        # ax1.hlines(y=0, xmin=0, xmax=len(self.distance_array), colors='black', linestyles='--', lw=1)
        # ax1.xaxis.set_major_formatter(format_fn)
        # ax1.xaxis.set_major_locator(MaxNLocator(integer=True))
        # ax1.set_xlabel('Followed distance (m)')
        # fig.legend(loc='right')




        # plt.axis('square')
        fig.legend(loc='center right')
        plt.suptitle('Alignment errors during autonomous cable following')
        plt.show()


        # fig, ((ax1, ax2, ax3, ax4)) = plt.subplots(nrows=4, ncols=1, sharex=True, sharey=False)
        # ax1.plot(self.all_x_error_, color='red', label='x')
        # ax2.plot(self.all_y_error_, color='green', label='y')
        # ax3.plot(self.all_z_error_, color='blue', label='z')
        # ax4.plot(self.all_yaw_error_, color='orange', label='yaw')

        # ax1.set_ylabel('X error (m)')
        # ax2.set_ylabel('Y error (m)')
        # ax3.set_ylabel('Z error (m)')
        # ax4.set_ylabel('Yaw error (rad)')

        # ax4.set_xlabel('time (10 ms)')

        # # fig.legend()

        # # plt.axis('square')
        # plt.suptitle('X, Y, Z, Yaw Errors During Autonomous Flight')
        # plt.show()



## PLOT ALL XYZ VALS WHEN CERTAIN KEY PRESSED


    ## Key press handling

    def press(self, key):
        
        if key == 'e' or key == 'enter':

            if len(self.pose_pair_array) < 1:
                print("Sequence has length 0, nothing to plot")

            else:
                self.draw_image()

        if key == 'space':
            self.vline_x_arr.append(len(self.pose_pair_array))
            print("Added vline at:",self.vline_x_arr[-1])

        else:
            print("Unsupported command:", key)
        

    def keylistener(self):

        print("Press 'e/enter' to plot sequence\n")

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