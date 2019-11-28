#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

ros_img = Image()

threshold_value = 250


def callback(data):
    global ros_img 
    ros_img  = data

def threshold(img_msg):

    bridge = CvBridge()

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "mono8")
        thresh, thresh_img = cv2.threshold(cv_image,threshold_value,255,cv2.THRESH_BINARY) 

        cv2.rectangle(thresh_img, (0,0), (640,128), (0,0,0), thickness=cv2.FILLED)  
        cv2.rectangle(thresh_img, (0,480), (640,320), (0,0,0), thickness=cv2.FILLED)    

        img_msg = bridge.cv2_to_imgmsg(thresh_img, "mono8") 
    except CvBridgeError as e:
      print(e)

    return img_msg

def threshold_publisher():
    global ros_img
    img_pub = rospy.Publisher('/sensors/camera/infra1/image_rect_threshold', Image, queue_size=10)
    rospy.init_node('threshold_publisher', anonymous=True)


    rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, callback)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        img_msg = Image()        
        img_msg = threshold(ros_img)
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