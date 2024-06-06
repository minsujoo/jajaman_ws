#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import itertools,operator

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped



prev_steering_angle = 0
t_prev = 0.0
prev_range = []

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers

        self.lidar_sub = rospy.Subscriber('/scan',LaserScan, self.lidar_callback) #None #TODO
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd',AckermannDriveStamped,queue_size=5) #TODO
    	

    def find_max_gap(self, free_space_ranges):
        # Return the start index & end index of the max gap in free_space_ranges
        

        return 0
    
    def find_best_point(self, start_i, end_i, ranges):
        # Start_i & end_i are start and end indicies of max-gap range, respectively
        # Return index of best point in ranges
	    # Naive: Choose the furthest point within ranges and go there
        

        return 0 


    # def find_ind(self, arr, threshold):
    #     start, end, max_len = 0, 0, 0
    #     for i in range(len(arr)):
    #         if arr[i] >= threshold:
    #             #
    #             if start == end:
    #                 start = i
    #             # 
    #             else:
    #                 end = i
    #         else:
    #             # 
    #             if end > start:
    #                 length = end - start + 1
    #                 if length > max_len:
    #                     max_len = length
    #                     max_start = start
    #                     max_end = end
    #             # 
    #             start = end = i
    #     # 
    #     if end > start:
    #         length = end - start + 1
    #         if length > max_len:
    #             max_len = length
    #             max_start = start
    #             max_end = end
    #     return (max_start + max_end)/2

    def lidar_callback(self, data):

        #TO DO:  
        # Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        #       1. Get LiDAR message
        #       2. Find closest point to LiDAR
        #       3. Eliminate all points inside 'bubble' (set them to zero) 
        #       4. Fine the max gap -> "def find_max_gap(self, free_space_ranges):"
        #       4. Find the best point -> "def find_best_point(self, start_i, end_i, ranges):"
        #       5. Publish Drive message

        a = []
        bubble = 0.3
        angle = 100
        midmin = int(2155/2 - angle * 5)
        midmax = int(2155/2 + angle * 5)
        # 
        for i in range(0, 2155):
            a.append(data.ranges[i])
        
        # 
        minv = np.argmin(a)
        ind = int(np.arctan(bubble/a[minv]) * 180 / np.pi * 4)
        #
        for i in range(minv - ind,minv + ind):
            if i < len(a):
                a[i] = 0


        # 
        threshold = 0.9
        start, end, max_len, max_start, max_end = 0, 0, 0, 0, 0
        for i in range(len(a)):
            if a[i] >= threshold:
                # 
                if start == end:
                    start = i
                # 
                else:
                    end = i
            else:
                # 
                if end > start:
                    length = end - start + 1
                    if length > max_len:
                        max_len = length
                        max_start = start
                        max_end = end
                # 
                start = end = i
        
        if end > start:
            length = end - start + 1
            if length > max_len:
                max_len = length
                max_start = start
                max_end = end

        # 
        gap = int((max_start + max_end)/2)
        print(gap)
        # 
        steering_angle = 1 * (gap - int(len(a)/2)) * data.angle_increment
        print(steering_angle)


        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.speed = 0.8
        self.drive_pub.publish(self.drive_msg)


def main(args):
    rospy.init_node("FollowGap_node_SJ", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
