#!/usr/bin/env python
import rospy
from autominy_msgs.msg import Tick, SteeringAngle
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import numpy as np

from timeit import default_timer as timer

time_last = timer()

meter_per_tick = 0.0033
axle_distance = 0.27


steering_angle = 0.0

pos_x = 0.0
pos_y = 0.0

accel_x = 0.0
accel_y = 0.0
accel_angel = 0.0

velocity = 0.0
angle = 0.0



def tick_callback(data):
    #print("Aktuelle Tick Value ist: " + str(data.value))
    global time_last
    global steering_angle
    global angle

    global pos_x
    global pos_y

    global velocity 
    global angle 

    time_now = timer()
    delta_time = time_now - time_last
    time_last = time_now

    distance = meter_per_tick * data.value
    velocity = distance / delta_time

    accel_x = velocity * np.cos(angle)
    accel_y = velocity * np.sin(angle)
    accel_angel = (velocity / axle_distance) * np.tan(steering_angle)

    pos_x = pos_x + (delta_time * accel_x)
    pos_y = pos_y + (delta_time * accel_y)
    angle = angle + (delta_time * accel_angel)


    #print("Geschwindigkeit " + str(velocity))
    

def steering_callback(data):
    #print("Aktuelle Steering Value ist: " + str(data.value))
    global steering_angle 
    steering_angle = data.value
    
def odom_node():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('odom_node', anonymous=True)

    rospy.Subscriber("/sensors/arduino/ticks", Tick, tick_callback)
    rospy.Subscriber("/sensors/steering", SteeringAngle, steering_callback)

    odom_broadcaster = tf.TransformBroadcaster()

    odom_pub = rospy.Publisher('/nav/Odometry', Odometry, queue_size=10)

    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        odom_msg = Odometry()


        odom_msg.pose.pose.position.x = pos_x
        odom_msg.pose.pose.position.y = pos_y
        odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        

        odom_pub.publish(odom_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        odom_node()
    except rospy.ROSInterruptException:
        pass