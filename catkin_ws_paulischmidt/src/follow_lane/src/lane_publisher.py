#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8

def target_lane_publisher():
    lane_pub = rospy.Publisher('/target_lane', UInt8, queue_size=10)
    rospy.init_node('target_lane_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        lane_msg = UInt8(1)        
        lane_pub.publish(lane_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        target_lane_publisher()
    except rospy.ROSInterruptException:
        pass