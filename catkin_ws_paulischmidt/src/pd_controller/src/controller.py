#!/usr/bin/env python
import rospy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import numpy as np
import sys

velocity = 0.2




steering_angle = 0.0

desired_angle = 0.0
last_error = 0.0

kd = 10.0
kp = 1.0



def odom_callback(data):

    global desired_angle
    global steering_angle
    global last_error

    orientation = data.pose.pose.orientation
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])    
    angle = yaw

    #print("Angle: " + str(angle))

    error = desired_angle - angle


    #print("angle: " + str(angle))
    print("error: " +  str(error))

    error_derivative = error - last_error

    control_value = kp * error + kd * error_derivative

    

    steering_angle = np.clip(control_value, -1.0 , 1.0)

    #print(steering_angle)

    last_error = error

    


def pd_node(arg):

    global desired_angle
    global velocity
    global steering_angle

    desired_angle = float(arg)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pd_node', anonymous=True)

    rospy.Subscriber("/sensors/localization/filtered_map", Odometry, odom_callback)

    steering_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
    speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)

    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        steering_msg = NormalizedSteeringCommand()
        #print(desired_angle)
        steering_msg.value = float(steering_angle)
        print("Steering: " + str(steering_angle))

        #print(float(desired_angle))

        speed_msg = SpeedCommand()
        speed_msg.value = velocity

        #print(velocity)
      

        steering_pub.publish(steering_msg)
        speed_pub.publish(speed_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        pd_node(rospy.myargv(argv=sys.argv)[1])
    except rospy.ROSInterruptException:
        pass