#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import random
import sys

threshold_img = Image()
raw_img = Image()

gl_pixeldistance = 5



def threshold_callback(data):
    global threshold_img 
    threshold_img  = data


def raw_callback(data):
    global raw_img 
    raw_img  = data

def get_pixels(img):
    pixel = []

    for x in range(0, 640):
        for y in range(0, 480):
            if(img[y,x][0] == 255 and img[y,x][1] == 255 and img[y,x][2] == 255):
                pixel.append((x,y))
    return pixel

def distance_to_line(point, m, n):
    return np.abs(m*point[0] + (-point[1]) + n ) / np.sqrt((m*m) + 1)

def ransac_lines(pixels, pixelpercentage):

    line_found = False

    

    while(not line_found):

        point1 = pixels[random.randrange(len(pixels)-1)]
        point2 = pixels[random.randrange(len(pixels)-1)]

        m = 0
        n = 0

        fit_pts = []

        while((m == 0)):
            point1 = pixels[random.randrange(len(pixels)-1)]
            point2 = pixels[random.randrange(len(pixels)-1)]

            if(point2[0] - point1[0] != 0):
                m = (float(point2[1] - point1[1]) / float(point2[0]- point1[0]))
                n = point2[1] - (m * point2[0])

        for pt in pixels:
            if(distance_to_line(pt, m ,n) < gl_pixeldistance):
                fit_pts.append(pt)

        if( float(len(fit_pts)) / float(len(pixels)) >= pixelpercentage):
            line_found = True

    return point1, point2, m, n, fit_pts






def line_detection(img_threshold, img_raw):

    bridge = CvBridge()

    m1 = 0.0
    m2 = 0.0
    m3 = 0.0

    n1 = 0.0
    n2 = 0.0
    n3 = 0.0

    try:
        cv_image = bridge.imgmsg_to_cv2(img_threshold, "rgb8")
        pixels = get_pixels(cv_image)

        raw_image_cv = bridge.imgmsg_to_cv2(img_raw, "rgb8")

        #print("Detected pixels: ")
        #print(len(pixels))

        point1, point2, m1 , n1, points = ransac_lines(pixels, 0.5)



        for x in points:
            cv2.circle(cv_image, x, 1, (0,0,0), -1)

        if(m1 != 0):
            p1 = (int((480-n1)/m1), 480)
            p2 = (int((-n1)/m1), 0)
        else:
            p1 = (0, n1)
            p2 = (640, n1)

        cv2.line(raw_image_cv, p1, p2,(0,255,0), 2)

        pixels = get_pixels(cv_image)

        point1, point2, m2 , n2, points = ransac_lines(pixels, 0.60)

        for x in points:
            cv2.circle(cv_image, x, 1, (0,0,0), -1)

        if(m2 != 0):
            p1 = (int((480-n2)/m2), 480)
            p2 = (int((-n2)/m2), 0)
        else:
            p1 = (0, n2)
            p2 = (640, n2)

        cv2.line(raw_image_cv, p1, p2,(0,255,0), 2)

        pixels = get_pixels(cv_image)

        point1, point2, m3 , n3, points = ransac_lines(pixels, 0.80)

        for x in points:
            cv2.circle(cv_image, x, 1, (0,0,0), -1)

        if(m3 != 0):
            p1 = (int((480-n3)/m3), 480)
            p2 = (int((-n3)/m3), 0)
        else:
            p1 = (0, n3)
            p2 = (640, n3)

        cv2.line(raw_image_cv, p1, p2,(0,255,0), 2)

        #cv2.circle(cv_image, point1, 10, (255,0,0), -1)

        #cv2.circle(cv_image, point2, 10, (255,0,0), -1)




  


        img_raw = bridge.cv2_to_imgmsg(raw_image_cv, "rgb8") 

    except CvBridgeError as e:
      print(e)

    return img_raw, m1, n1, m2, n2, m3, n3

def threshold_publisher():
    global ros_img
    img_pub = rospy.Publisher('/sensors/camera/infra1/image_rect_lines', Image, queue_size=10)

    m1_pub = rospy.Publisher('/sensors/camera/line_detection/line1/m', Float32, queue_size=10)
    m2_pub = rospy.Publisher('/sensors/camera/line_detection/line2/m', Float32, queue_size=10)
    m3_pub = rospy.Publisher('/sensors/camera/line_detection/line3/m', Float32, queue_size=10)

    n1_pub = rospy.Publisher('/sensors/camera/line_detection/line1/n', Float32, queue_size=10)
    n2_pub = rospy.Publisher('/sensors/camera/line_detection/line2/n', Float32, queue_size=10)
    n3_pub = rospy.Publisher('/sensors/camera/line_detection/line3/n', Float32, queue_size=10)

    rospy.init_node('markert_detection', anonymous=True)


    rospy.Subscriber('/sensors/camera/infra1/image_rect_threshold', Image, threshold_callback)
    rospy.Subscriber('/sensors/camera/infra1/image_rect_raw', Image, raw_callback)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        img_msg = Image()        
        

        m1_msg = Float32()
        n1_msg = Float32()

        m2_msg = Float32()
        n2_msg = Float32()

        m3_msg = Float32()
        n3_msg = Float32()

        img_msg, m1_msg, n1_msg, m2_msg, n2_msg, m3_msg, n3_msg = line_detection(threshold_img, raw_img)
#        img_msg.header = bild.header
 #       img_msg.height = bild.height
#        img_msg.width = bild.width
#        img_msg.encoding = bild.encoding
#        img_msg.is_bigendian = bild.is_bigendian
#        img_msg.step = bild.step
#        img_msg.data = bild.data

        
        img_pub.publish(img_msg)
        m1_pub.publish(m1_msg)
        m2_pub.publish(m2_msg)
        m2_pub.publish(m3_msg)

        n1_pub.publish(n1_msg)
        n2_pub.publish(n2_msg)
        n3_pub.publish(n3_msg)

        rate.sleep()

    

if __name__ == '__main__':
    try:
        threshold_publisher()
    except rospy.ROSInterruptException:
        pass