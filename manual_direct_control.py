'''
Publish '/vesc/ackermann_cmd' filled with user inputs 
for 'vesc' in jetson to control the motors
'''

import sys
import math
import numpy as np
import rospy

from ackermann_msgs.msg import AckermannDriveStamped


def main():
    rospy.init_node("manual_drive_node", anonymous=True)
    
    drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped,queue_size=5)
    
    steer_deg = 0.0
    speed = 0.0
    steer_deg = float(input("steer in degree?   ")) - 1.5
    steer = steer_deg / 180 * math.pi
    speed = float(input("speed?   "))

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = steer * (-1)
        drive_msg.drive.speed = speed
        drive_pub.publish(drive_msg)
        print("\n\n\n")
        print("speed       : ", speed)
        print("steer in rad: ", steer)
        print("steer in deg: ", steer_deg)

        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass