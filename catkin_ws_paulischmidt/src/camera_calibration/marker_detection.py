#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

ros_img = Image()

mrk1_pt1 = (197,231)
mrk1_pt2 = (217,247)

mrk2_pt1 = (491,217)
mrk2_pt2 = (516,231)

mrk3_pt1 = (240,157)
mrk3_pt2 = (254,168)

mrk4_pt1 = (438,147)
mrk4_pt2 = (454,155)

mrk5_pt1 = (262,119)
mrk5_pt2 = (274,126)

mrk6_pt1 = (411,110)
mrk6_pt2 = (423,117)

def callback(data):
    global ros_img 
    ros_img  = data

def get_pixels( corner_1 , corner_2, img):
    pixel = []

    for x in range(corner_1[0]+1, corner_2[0]):
        for y in range(corner_1[1]+1, corner_2[1]):
            if(img[y,x][1] == 255):
                pixel.append((x,y))


    return pixel

def average_pixel(pixels):

    num = len(pixels)

    x = 0
    y = 0

    for i in range(0, num):
        x += pixels[i][0]
        y += pixels[i][1]

    return (x//num, y//num)



def marker_detection(img_msg):

    bridge = CvBridge()

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "rgb8")
  
        cv2.rectangle(cv_image, mrk1_pt1, mrk1_pt2, (255,0,0), thickness=1, lineType=8, shift=0)
        cv2.rectangle(cv_image, mrk2_pt1, mrk2_pt2, (255,0,0), thickness=1, lineType=8, shift=0)
        cv2.rectangle(cv_image, mrk3_pt1, mrk3_pt2, (255,0,0), thickness=1, lineType=8, shift=0)
        cv2.rectangle(cv_image, mrk4_pt1, mrk4_pt2, (255,0,0), thickness=1, lineType=8, shift=0)
        cv2.rectangle(cv_image, mrk5_pt1, mrk5_pt2, (255,0,0), thickness=1, lineType=8, shift=0)
        cv2.rectangle(cv_image, mrk6_pt1, mrk6_pt2, (255,0,0), thickness=1, lineType=8, shift=0)

        marker1_location = average_pixel(get_pixels(mrk1_pt1,mrk1_pt2,cv_image))
        cv_image[marker1_location[1],marker1_location[0]]=(0,255,0)

        marker2_location = average_pixel(get_pixels(mrk2_pt1,mrk2_pt2,cv_image))
        cv_image[marker2_location[1],marker2_location[0]]=(0,255,0)

        marker3_location = average_pixel(get_pixels(mrk3_pt1,mrk3_pt2,cv_image))
        cv_image[marker3_location[1],marker3_location[0]]=(0,255,0)

        marker4_location = average_pixel(get_pixels(mrk4_pt1,mrk4_pt2,cv_image))
        cv_image[marker4_location[1],marker4_location[0]]=(0,255,0)

        marker5_location = average_pixel(get_pixels(mrk5_pt1,mrk5_pt2,cv_image))
        cv_image[marker5_location[1],marker5_location[0]]=(0,255,0)

        marker6_location = average_pixel(get_pixels(mrk6_pt1,mrk6_pt2,cv_image))
        cv_image[marker6_location[1],marker6_location[0]]=(0,255,0)


        img_msg = bridge.cv2_to_imgmsg(cv_image, "rgb8") 

        camera_mat = np.float64([[383.7944641113281, 0.0, 322.3056945800781], [0.0, 383.7944641113281, 241.67051696777344],[ 0.0, 0.0, 1.0]])

        camera_points = np.float32([[marker1_location[0],marker1_location[1]],
                                    [marker2_location[0],marker2_location[1]],
                                    [marker3_location[0],marker3_location[1]],
                                    [marker4_location[0],marker4_location[1]],
                                    [marker5_location[0],marker5_location[1]],
                                    [marker6_location[0],marker6_location[1]]])

        real_world_points = np.float32([[0.5,0.2,0],[0.5,-0.2,0],[0.8,0.2,0],[0.8,-0.2,0],[1.1,0.2,0],[1.1,-0.2,0]])

        distCoeffs = np.zeros(5)


        ret, rvec, tvec = cv2.solvePnP(real_world_points, camera_points, camera_mat, distCoeffs)

        

        rotmatrix, jacobian = cv2.Rodrigues(rvec)

        print("Rotatiosnvector: ", rvec)
        print("Rotationsmatrix", rotmatrix)
        print("Translationsvector: ", tvec)

    except CvBridgeError as e:
      print(e)

    return img_msg

def threshold_publisher():
    global ros_img
    img_pub = rospy.Publisher('/sensors/camera/infra1/image_rect_threshold_markers', Image, queue_size=10)
    rospy.init_node('markert_detection', anonymous=True)


    rospy.Subscriber('/sensors/camera/infra1/image_rect_threshold', Image, callback)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        img_msg = Image()        
        img_msg = marker_detection(ros_img)
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