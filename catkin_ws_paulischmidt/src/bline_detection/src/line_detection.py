#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import random
import sys

ros_img = Image()

gl_pixeldistance = 10
gl_pixelpercentage = 0.2


def callback(data):
    global ros_img 
    ros_img  = data

def get_pixels(img):
    pixel = []

    for x in range(0, 640):
        for y in range(0, 480):
            if(img[y,x][1] == 255):
                pixel.append((x,y))
    return pixel

def ransac_lines(pixels):

    line_found = False

    numpixels = len(pixels) * gl_pixelpercentage

    while(not line_found):

        point1 = pixels[random.randrange(len(pixels)-1)]
        point2 = pixels[random.randrange(len(pixels)-1)]

        print(point1)
        print(point2)

        if(point2[0] - point1[0] != 0):
            m = (point2[1] - point1[1]) / (point2[0]- point1[0])
        else:
            m = sys.float_info.max

        n = point2[1] - (m * point2[0])

        line_found = True

    return point1, point2, m ,n





def line_detection(img_msg):

    bridge = CvBridge()

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "rgb8")

        pixels = get_pixels(cv_image)

        #print("Detected pixels: ")
        #print(len(pixels))

        point1, point2, m ,n = ransac_lines(pixels)

        cv2.circle(cv_image, point1, 2, (128,0,0), -1)

        cv2.circle(cv_image, point2, 2, (128,0,0), -1)
  


        img_msg = bridge.cv2_to_imgmsg(cv_image, "rgb8") 

    except CvBridgeError as e:
      print(e)

    return img_msg

def threshold_publisher():
    global ros_img
    img_pub = rospy.Publisher('/sensors/camera/infra1/image_rect_lines', Image, queue_size=10)
    rospy.init_node('markert_detection', anonymous=True)


    rospy.Subscriber('/sensors/camera/infra1/image_rect_threshold', Image, callback)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        img_msg = Image()        
        img_msg = line_detection(ros_img)
#        img_msg.header = bild.header
 #       img_msg.height = bild.height
#        img_msg.width = bild.width
#        img_msg.encoding = bild.encoding
#        img_msg.is_bigendian = bild.is_bigendian
#        img_msg.step = bild.step
#        img_msg.data = bild.data

        img_pub.publish(img_msg)
        rate.sleep()

    

if __name__ == '__main__':
    try:
        threshold_publisher()
    except rospy.ROSInterruptException:
        pass