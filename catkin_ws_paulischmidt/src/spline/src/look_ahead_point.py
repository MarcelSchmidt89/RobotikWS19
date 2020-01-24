#!/usr/bin/env python
import rospy
import numpy  as np
import os
from scipy.interpolate import CubicSpline
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import sys

clicked_point = np.array([0.0, 0.0]) 

def distance(point1, point2):
    return(np.linalg.norm(point1-point2))


def lookahead_point(spline_x, spline_y, range,  point, offset):

    min_range = 0
    max_range = range

    dist = 99999

    i = 10000

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

def click_callback(data):
    global clicked_point
    clicked_point[0] = data.point.x
    clicked_point[1] = data.point.y

def spline_publisher():

    global clicked_point

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

    sample_points = lane1_array[[0, 25, 50, 75, 209, 225, 259, 275, 309, 325, 350, 375, 409, 509, 575, 639, 750, 800, 848, 900, 948, 975, 1028, 1148, 1200, 1276], :]
    sample_points_np = np.array(sample_points)
    lane1_arc_array = sample_points_np[:,0]
    lane1_x_array = sample_points_np[:,1]
    lane1_y_array = sample_points_np[:,2]

    lane1_x_spline = CubicSpline(lane1_arc_array, lane1_x_array, bc_type='periodic')
    lane1_y_spline = CubicSpline(lane1_arc_array, lane1_y_array, bc_type='periodic')

#### Lane 2

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

        
    
    while not rospy.is_shutdown():

        look_ahead_point = lookahead_point(lane1_x_spline, lane1_y_spline, 12.80, clicked_point, 0.1) 

        marker_strip = init_Marker_Strip(lane1_x_spline, lane1_y_spline)

        # Publish the Marker
        spline_pub.publish(marker_strip)

        click_marker = init_sphere_Marker(clicked_point, [1.0,0.0,1.0])

        # Publish the Marker
        click_pub.publish(click_marker)


        point_marker = init_sphere_Marker(look_ahead_point, [1.0,0.0,0.0])

        # Publish the Marker
        lookahead_point_pub.publish(point_marker)
     
        rate.sleep()

    

if __name__ == '__main__':
    try:
        spline_publisher()
    except rospy.ROSInterruptException:
        pass