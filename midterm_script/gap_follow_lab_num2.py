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

#AEB PARAMS
WATCHING_ANGLE = 40.0
SAFETY_TIME = 1.0

#GAP FOLLOW PARAMS
threshold = 0.9
VELOCITY = 0.2   # meters per second



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
    
    
    def find_longest_sequence_of_ones(self, lst):
        max_length = 0
        current_length = 0
        start_index = -1
        end_index = -1
        max_start_index = -1
        max_end_index = -1

        for i, value in enumerate(lst):
            if value == 1:
                if current_length == 0:  # Start of a new sequence
                    start_index = i
                current_length += 1
                end_index = i
            else:
                if current_length > max_length:  # Found a longer sequence
                    max_length = current_length
                    max_start_index = start_index
                    max_end_index = end_index
                current_length = 0  # Reset for the next sequence

        # Check if the last sequence is the longest
        if current_length > max_length:
            max_start_index = start_index
            max_end_index = end_index

        return (max_start_index, max_end_index)


    def lidar_callback(self, data):

        #TO DO:  
        # Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        #       1. Get LiDAR message
        #       2. Find closest point to LiDAR
        #       3. Eliminate all points inside 'bubble' (set them to zero) 
        #       4. Fine the max gap -> "def find_max_gap(self, free_space_ranges):"
        #       4. Find the best point -> "def find_best_point(self, start_i, end_i, ranges):"
        #       5. Publish Drive message

        ################################################################
        ### HERE IS FOR AEB

        watching_angle = WATCHING_ANGLE  # degrees
        watching_angle = watching_angle * math.pi / 180  # rad
        watching_channels = int(watching_angle / data.angle_increment)

        min_dist = min(data.ranges[int(len(data.ranges)/2 - watching_channels/2) : int(len(data.ranges)/2 + watching_channels/2)])

        ttc = min_dist / VELOCITY  # sec

        if ttc < SAFETY_TIME:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.speed = 0
            self.drive_pub.publish(drive_msg)
            print("AEB working!")
            print("AEB working!")
            print("AEB working!")

        ################################################################

        else:
            gap_array = np.zeros(len(data.ranges))
            for i in range(len(data.ranges)):
                if data.ranges[i] > threshold:
                    gap_array[i] = 1

            gap_left_right = self.find_longest_sequence_of_ones(gap_array)
            gap_left = gap_left_right[0]
            gap_right = gap_left_right[1]
            gap_center = (gap_left + gap_right) / 2

            steering_input = (len(data.ranges) / 2 - gap_center) * data.angle_increment # rad # to the right




        
            self.drive_msg = AckermannDriveStamped()
            self.drive_msg.header.stamp = rospy.Time.now()
            self.drive_msg.drive.steering_angle = steering_input * (-1)
            self.drive_msg.drive.speed = VELOCITY
            self.drive_pub.publish(self.drive_msg)
        
        # print("steering: ", steering_input)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
