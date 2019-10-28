#!/usr/bin/env python
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand

def test_publischer():
    str_pub = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    speed_pub = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    rospy.init_node('test_publischer', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        str_msg = NormalizedSteeringCommand()
        str_msg.value = 1.0

        speed_msg = SpeedCommand()
        speed_msg.value = 0.3

        str_pub.publish(str_msg)
        speed_pub.publish(speed_msg)
        rate.sleep()

    

if __name__ == '__main__':
    try:
        test_publischer()
    except rospy.ROSInterruptException:
        pass