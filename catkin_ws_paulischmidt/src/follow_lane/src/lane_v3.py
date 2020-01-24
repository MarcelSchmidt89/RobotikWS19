#!/usr/bin/env python
import rospy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import UInt8
import tf
import numpy as np
import sys
from timeit import default_timer as timer

import os
from scipy.interpolate import CubicSpline
from geometry_msgs.msg import Point, PointStamped
import sys

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped

#velocity = 0.2

steering_angle = 0.0

#desired_angle = 0.0
last_error = 0.0

kd = 1.0
kp = 2.0

#desired_point = np.array([5.94,4.26])

position = [0,0]

marker_point = np.array([0,0])

last_seq = 0

target_lane = 2

#### Lane 1

script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
rel_path = "data/lane1.npy"
lane1_file_path = os.path.join(script_dir, rel_path)

rel_path = "data/lane1.npy"
lane1_file_path = os.path.join(script_dir, rel_path)
lane1_array = np.load(lane1_file_path)

offset = int(len(lane1_array) / 20) 

sample_points = lane1_array[[0, 25, 50, 75, 209, 225, 259, 275, 309, 325, 350, 375, 409, 509, 575, 639, 750, 800, 848, 900, 948, 975, 1028, 1148, 1200, 1276], :]
sample_points_np = np.array(sample_points)
lane1_arc_array = sample_points_np[:,0]
lane1_x_array = sample_points_np[:,1]
lane1_y_array = sample_points_np[:,2]

lane1_x_spline = CubicSpline(lane1_arc_array, lane1_x_array, bc_type='periodic')
lane1_y_spline = CubicSpline(lane1_arc_array, lane1_y_array, bc_type='periodic')

### Lane 2

rel_path = "data/lane2.npy"
lane1_file_path = os.path.join(script_dir, rel_path)
lane2_array = np.load(lane1_file_path)

sample_points = lane2_array[[0, 25, 50, 75, 100, 125, 150, 209, 400, 500, 600, 738, 800, 825, 850, 875, 900, 925, 949, 1150, 1300, 1476], :]
sample_points_np = np.array(sample_points)
lane2_arc_array = sample_points_np[:,0]
lane2_x_array = sample_points_np[:,1]
lane2_y_array = sample_points_np[:,2]

lane2_x_spline = CubicSpline(lane2_arc_array, lane2_x_array, bc_type='periodic')
lane2_y_spline = CubicSpline(lane2_arc_array, lane2_y_array, bc_type='periodic')

target_speed = 0.2


straight_speed = 0.5



def odom_callback(data):

    global desired_angle
    global steering_angle
    global last_error
    global position
#    global desired_point
    global marker_point
    global last_seq
    global target_lane
    global target_speed

    seq = data.header.seq

    #print(target_lane)

    if(seq > last_seq):        

        position[0] = data.pose.pose.position.x
        position[1] = data.pose.pose.position.y
        orientation = data.pose.pose.orientation

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])    
        angle = yaw

        if(target_lane == 1):
            desired_point = lookahead_point(lane1_x_spline, lane1_y_spline, 12.80, np.array(position), 0.5)
            curve_check = lookahead_point(lane1_x_spline, lane1_y_spline, 12.80, np.array(position), 1.0)
        else:
            desired_point = lookahead_point(lane2_x_spline, lane2_y_spline, 14.76, np.array(position), 0.5)
            curve_check = lookahead_point(lane2_x_spline, lane2_y_spline, 14.76, np.array(position), 1.0)


        marker_point = desired_point
        desired_point = desired_point - position
        curve_check = curve_check - position        
     
        rotmat = np.array(( (np.cos(-angle), -np.sin(-angle)),
                   (np.sin(-angle),  np.cos(-angle)) ))

        desired_point_carview = rotmat.dot(np.array(desired_point)) 
        curve_check_carview = rotmat.dot(np.array(curve_check))

        desired_angle = np.arctan(desired_point_carview[1] / desired_point_carview[0])
        curve_check_angle = np.arctan(curve_check_carview[1] / curve_check_carview[0])

        speed_coef = np.clip(np.abs(desired_angle-curve_check_angle) * 10,0.0, 1.0)

        print(speed_coef)
        target_speed = straight_speed - ((speed_coef * 3) / 10)
        print(target_speed)

        error = desired_angle

        error_derivative = error - last_error 

        control_value = kp * error + kd * error_derivative        

        steering_angle = control_value
        last_error = error

    last_seq = seq

def lane_callback(data):
    global target_lane
    target_lane = data.data

def distance(point1, point2):
    return(np.linalg.norm(point1-point2))


def lookahead_point(spline_x, spline_y, range,  point, offset):

    min_range = 0
    max_range = range

    dist = 99999

    i = 10

    while(i > 0):

        range_mid = (min_range + max_range) / 2

        half_dist = np.abs(range_mid - min_range) / 2

        point_low = np.array([spline_x(range_mid - half_dist), spline_y(range_mid - half_dist)])
        point_high = np.array([spline_x(range_mid + half_dist), spline_y(range_mid + half_dist)])

        #print(point_low)
        #print(point_high)

        if(distance(point_low, point) < distance(point_high, point)):
            max_range = range_mid
            result = [point_low[0], point_low[1]]
            dist = distance(result, point)

        else:
            min_range = range_mid
            result = [point_high[0], point_high[1]]
            dist = distance(result, point)

        i = i-1

    return np.array([spline_x(range_mid + offset), spline_y(range_mid + offset)])


def init_sphere_Marker(Point, RGBColor):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # marker color
        marker.color.a = 1.0
        marker.color.r = RGBColor[0]
        marker.color.g = RGBColor[1]
        marker.color.b = RGBColor[2]

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = Point[0]
        marker.pose.position.y = Point[1]
        marker.pose.position.z = 0.0

        # marker line points
        marker.points = []

        return marker


def init_Marker_Strip(xSpline, ySpline):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.05

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker line points
        marker.points = []

        for i in range(1280):
            point = Point()
            point.x = xSpline(float(i)/100)
            point.y = ySpline(float(i)/100)
            point.z = 0.0
            marker.points.append(point)

        return marker


def controll_node(arg):

    global desired_angle
    global velocity
    global steering_angle
    global desired_point
    global marker_point
    global target_speed



    #desired_angle = float(arg)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('controll_node', anonymous=True)

    rospy.Subscriber("/sensors/localization/filtered_map", Odometry, odom_callback)

    rospy.Subscriber("/target_lane", UInt8, lane_callback)

    steering_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
    speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)

    closest_point_pub = rospy.Publisher('closest', Marker, queue_size = 1)

    spline_pub = rospy.Publisher('spline', Marker, queue_size=1)

    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():


        steering_msg = NormalizedSteeringCommand()
        #print(desired_angle)
        steering_msg.value = float(steering_angle)
        #print("Steering: " + str(steering_angle))

        #print(float(desired_angle))

        speed_msg = SpeedCommand()
        speed_msg.value = target_speed 

        #print(velocity)
      

        steering_pub.publish(steering_msg)
        speed_pub.publish(speed_msg)

        point_marker = init_sphere_Marker(marker_point, [1.0,0.0,0.0])
        closest_point_pub.publish(point_marker)

        #marker_strip = init_Marker_Strip(lane1_x_spline, lane1_y_spline)

        #spline_pub.publish(marker_strip)

        rate.sleep()


if __name__ == '__main__':
    try:
        controll_node(rospy.myargv(argv=sys.argv))
    except rospy.ROSInterruptException:
        pass