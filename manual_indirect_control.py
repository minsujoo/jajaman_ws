'''
Publish '/steer' and '/speed' filled with user inputs 
for 'vesc_commander' to publish '/vesc/ackermann_cmd' with user-selelted steering and speed
'''

import sys
import math
import numpy as np
import rospy

from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped


def main():
    rospy.init_node("manual_drive_node", anonymous=True)
    
    # drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped,queue_size=5)
    steer_pub = rospy.Publisher('/steer', Float64,queue_size=5)
    speed_pub = rospy.Publisher('/speed', Float64,queue_size=5)
    
    
    steer_deg = 0.0
    speed = 0.0
    # steer_deg = float(input("steer in degree?   ")) - 1.5
    steer_deg = float(input("steer in degree?   "))
    steer = steer_deg / 180 * math.pi
    speed = float(input("speed?   "))

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        '''
        steer = 0
        speed = 0
        print("steer?")
        input(steer)
        print("speed?")
        input(speed)
        '''
        # drive_msg = AckermannDriveStamped()
        # drive_msg.header.stamp = rospy.Time.now()
        # drive_msg.drive.steering_angle = steer * (-1)
        # drive_msg.drive.speed = speed
        # drive_pub.publish(drive_msg)
        steer_msg = Float64()
        steer_msg.data = steer
        steer_pub.publish(steer_msg)
        speed_msg = Float64()
        speed_msg.data = speed
        speed_pub.publish(speed_msg)
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