import sys
import math
import numpy as np
import rospy

from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


def main():
    rospy.init_node("manual_drive_node", anonymous=True)
    
    # drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped,queue_size=5)
    e_stop_pub = rospy.Publisher('/e_stop', Bool,queue_size=5)
    
    '''
    e_stop_int = input("0: no e_stop\n1: e_stop, FULL BRAKE\nwhat do you want?")
    if e_stop_int == "0":
        e_stop = False
    elif e_stop_int == "1":
        e_stop = True
    else:
        e_stop = True
    '''

    e_stop = True

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        e_stop_msg = Bool()
        e_stop_msg.data = e_stop
        e_stop_pub.publish(e_stop_msg)
        print("e_stop : ", e_stop)

        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass