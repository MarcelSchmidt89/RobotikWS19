#!/usr/bin/env python
import rospy
import numpy  as np
import os
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
from scipy.interpolate import CubicSpline

def test_publischer():
    str_pub = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    speed_pub = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    rospy.init_node('test_publischer', anonymous=True)
    rate = rospy.Rate(10) # 10hz


    script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
    rel_path = "data/lane1.npy"
    lane1_file_path = os.path.join(script_dir, rel_path)

    
    #np.save('hiyop1337', np.array([[1, 2, 3], [4, 5, 6]]))
    lane1_array = np.load(lane1_file_path)

    arc_array = lane1_array[:0]
    x_array = lane1_array[:1]
    y_array = lane1_array[:2]

    x_spline = CubicSpline(arc_array, x_array)

    y_spline = CubicSpline(arc_array, y_array)

    

    
    # while not rospy.is_shutdown():
    #     str_msg = NormalizedSteeringCommand()
    #     str_msg.value = 1.0

    #     speed_msg = SpeedCommand()
    #     speed_msg.value = 0.3

    #     str_pub.publish(str_msg)
    #     speed_pub.publish(speed_msg)
    #     rate.sleep()

    

if __name__ == '__main__':
    try:
        test_publischer()
    except rospy.ROSInterruptException:
        pass