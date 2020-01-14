#!/usr/bin/env python
import rospy
import numpy  as np
import os
from scipy.interpolate import CubicSpline
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys

def distance(point1, point2):
    return(np.linalg.norm(point1-point2))

def spline_publisher():
    rospy.init_node('spline_publisher')
    spline_pub = rospy.Publisher('spline', Marker, queue_size=1)

    spline_pub2 = rospy.Publisher('spline2', Marker, queue_size=1)
    
    rate = rospy.Rate(30) # 10hz


    script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
    rel_path = "data/lane1.npy"
    lane1_file_path = os.path.join(script_dir, rel_path)


#### Lane 1

    rel_path = "data/lane1.npy"
    lane1_file_path = os.path.join(script_dir, rel_path)

    lane1_array = np.load(lane1_file_path)

    offset = int(len(lane1_array) / 20) 

    sample_points = []
 
    for i in range(20):
        index = i * offset
        sample_points.append(lane1_array[index])

    sample_points.append(lane1_array[-1])


    sample_points_np = np.array(sample_points)    


    lane1_arc_array = sample_points_np[:,0]
    lane1_x_array = sample_points_np[:,1]
    lane1_y_array = sample_points_np[:,2]

    print(lane1_arc_array)

    lane1_x_spline = CubicSpline(lane1_arc_array, lane1_x_array)

    lane1_y_spline = CubicSpline(lane1_arc_array, lane1_y_array)

#### Lane 2

    rel_path = "data/lane2.npy"
    lane1_file_path = os.path.join(script_dir, rel_path)

    lane2_array = np.load(lane1_file_path)

    offset = int(len(lane2_array) / 20)

    sample_points = []
 
    for i in range(20):
        index = i * offset
        sample_points.append(lane2_array[index])

    sample_points.append(lane2_array[-1])


    sample_points_np = np.array(sample_points)    


    lane2_arc_array = sample_points_np[:,0]
    lane2_x_array = sample_points_np[:,1]
    lane2_y_array = sample_points_np[:,2]

    print(lane1_arc_array)

    print(lane2_array[-1])

    lane2_x_spline = CubicSpline(lane2_arc_array, lane2_x_array)

    lane2_y_spline = CubicSpline(lane2_arc_array, lane2_y_array)




    
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

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
            point.x = lane1_x_spline(float(i)/100)
            point.y = lane1_y_spline(float(i)/100)
            point.z = 0.0
            marker.points.append(point)

        # Publish the Marker
        spline_pub.publish(marker)


        marker2 = Marker()
        marker2.header.frame_id = "map"
        marker2.type = marker.LINE_STRIP
        marker2.action = marker.ADD

        # marker scale
        marker2.scale.x = 0.01
        marker2.scale.y = 0.01
        marker2.scale.z = 0.01

        # marker color
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0

        # marker orientaiton
        marker2.pose.orientation.x = 0.0
        marker2.pose.orientation.y = 0.0
        marker2.pose.orientation.z = 0.0
        marker2.pose.orientation.w = 1.0

        # marker position
        marker2.pose.position.x = 0.0
        marker2.pose.position.y = 0.0
        marker2.pose.position.z = 0.0

        # marker line points
        marker2.points = []

        for i in range(1476):
            point = Point()
            point.x = lane2_x_spline(float(i)/100)
            point.y = lane2_y_spline(float(i)/100)
            point.z = 0.0
            marker2.points.append(point)

        # Publish the Marker
        spline_pub2.publish(marker2)

     
        rate.sleep()

    

if __name__ == '__main__':
    try:
        spline_publisher()
    except rospy.ROSInterruptException:
        pass