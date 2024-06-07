#!/usr/bin/env python
import sys
import math
import numpy as np  

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

#TO DO: Tune parameters
#PARAMS 
VELOCITY = 1.0 # meters per second
MIN_ACC = 1.0

WATCHING_ANGLE = 30.0
SAFETY_TIME = 1.0


class EmergencyStop:
    def __init__(self):
        # Topics & Subs, Pubs
        self.stop_signal = 0
        self.velocity_sub = rospy.Subscriber("/vesc/odom", Odometry, self.callback_vel)#: Subscribe to VESC
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.callback)#: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped, queue_size=10)#: Publish to drive

    def callback_vel(self,data):
        # TO DO: Subscribe Current Velocity
        # self.x_vel = data.twist.twist.linear.x
        self.vel = VELOCITY

    def callback(self, data):
        #TO DO: 1. Subscribe LiDAR data 
        #       2. Calculate minimum distance 
        #       2. Calculate Time to Collision(TTC) based on current velocity
        #       3. Publish drive.speed. (If TTC is smaller than minimum time, stop the car )
        #       
        #       Example:
        #               drive_msg = AckermannDriveStamped()
        #               drive_msg.header.stamp = rospy.Time.now()
        #               drive_msg.header.frame_id = "laser"
        #               drive_msg.drive.speed = 0
        #               self.drive_pub.publish(drive_msg)
        self.lidar_data = data
        
        # print("lidar data received!")
        # print(len(data.ranges))

        watching_angle = WATCHING_ANGLE  # degrees
        watching_angle = watching_angle * math.pi / 180  # rad
        watching_channels = int(watching_angle / data.angle_increment)
        print(watching_channels)
        print(len(data.ranges)/2 - watching_channels/2)

        min_dist = min(data.ranges[int(len(data.ranges)/2 - watching_channels/2) : int(len(data.ranges)/2 + watching_channels/2)])

        ttc = min_dist / VELOCITY  # sec

        if ttc < SAFETY_TIME:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.speed = 0
            self.drive_pub.publish(drive_msg)
        
        else:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.speed = VELOCITY
            self.drive_pub.publish(drive_msg)

         

        
    
def main(args):
    rospy.init_node("Emergengy_Stop", anonymous=False)
    ES = EmergencyStop()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)