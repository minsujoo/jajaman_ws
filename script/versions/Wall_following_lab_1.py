### Wall following without AEB

#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped



#TO DO: Tune parameters
#PID CONTROL PARAMS
kp = 0.4
kd = 0.1
ki = 0.0
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS 
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.7 # meters
DESIRED_DISTANCE_LEFT = 1.0
VELOCITY = 0.3 # meters per second
CAR_LENGTH = 0.50 # TfollowLeftraxxas Rally is 20 inches or 0.5 meters


class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)#: Subscribe to LIDAR
        # self.drive_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=10)#: Publish to drive
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped, queue_size=10)#: Publish to drive


    # def getRange(self, data):
    #     # data: single message from topic /scan
    #     # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
    #     # Outputs length in meters to object with angle in lidar scan field of view
    #     #make sure to take care of nans etc.
    #     #TODO: implement
    #     laser_ranges = data.ranges
    #     laser_ranges = laser_ranges[600:710]
    #     return 0.0


    def pid_control(self, error, velocity, time_gap):

        global integral
    
        #TODO: Use kp, ki & kd to implement a PID controller
        #       Example:
        #               drive_msg = AckermannDriveStamped()
        #               drive_msg.header.stamp = rospy.Time.now()
        #               drive_msg.header.frame_id = "laser"
        #               drive_msg.drive.speed = 0
        #               self.drive_pub.publish(drive_msg)
         
        p_value = kp * error

        integral += error
        i_value = ki * integral

        derivative = (error - prev_error) / time_gap
        d_value = kd * derivative

        print("error: ", error)
        print("p_value: ", p_value)
        print("i_value: ", i_value)
        print("d_value: ", d_value)
        print("p+i+d", p_value + i_value + d_value)
        print("\n")

        return p_value + i_value + d_value
    

    def lidar_callback(self, data):

        #TODO:  
        #       1. Get LiDAR message
        #       2. Calculate length to object with angle in lidar scan field of view
        #          and make sure to take care of nans etc.
        #       3. Based on length to object, callback 'pid_control' 
        self.lidar_data = data
        range_a = data.ranges[710]
        range_b = data.ranges[600]
        theta = (710 - 600) * data.angle_increment  # radian
        alpha = np.arctan((range_a*np.cos(theta) - range_b) / (range_a * np.sin(theta)))
        len_ab = range_b * np.cos(alpha)
        len_ac = 0.1  # why??
        error = len_ab + len_ac * np.sin(alpha) - DESIRED_DISTANCE_RIGHT

        controller_output = self.pid_control(error, VELOCITY, data.scan_time)

        print("controller_output: ", controller_output)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.speed = VELOCITY
        drive_msg.drive.steering_angle = -1 * controller_output
        self.drive_pub.publish(drive_msg)



def main(args):

    rospy.init_node("WallFollow_node", anonymous=False)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()


if __name__=='__main__':
	main(sys.argv)
