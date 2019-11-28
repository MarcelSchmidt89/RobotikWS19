#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand, SteeringFeedback, Tick
import numpy as np

measurement1 = (0,0,0)
measurement2 = (0,0,0)
measurement3 = (0,0,0)

start = False

position = (0,0,0)

target_speed = 0

first = True

axe_distance = 0.27

steering_feedback = 0

tick = 0

def tick_callback(data):
    global tick
    tick += data.value



def callback(data):
    #print(data.pose.pose.orientation)
    #quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w )
    #euler = euler_from_quaternion(quaternion)
    global position
    global start
    global first
    

    if first:
        start = True
        first = False
        print("started control sequence")
    position = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)

    #print(abs(data.pose.pose.orientation.z))

def control_handler(seconds_passed, speed_pub):
    global start
    global measurement1, measurement2, measurement3
    global target_speed
    global steering_feedback
    #global position
    target_speed = 0
    #print(seconds_passed)
    #print(start)
    if start:
        
        
        #print(position)
        #measurement1 = data.pose.pose.position
        if(seconds_passed > 10.0 ):
            target_speed = 0.2
            #print(seconds_passed)
            #print(measurement1)
            if(measurement1 == (0,0,0)):
                measurement1 = position
                print("First measurement: ")
                print(measurement1)              
            

        if(seconds_passed > 12.0):
            if(measurement2 == (0,0,0)):
                measurement2 = position
                print("Second measurement: ")
                print(measurement2)

        if(seconds_passed > 14.0):
            if(measurement3 == (0,0,0)):
                measurement3 = position
                print("Third measurement: ")
                print(measurement3)
                #print("set speed to 0.15")
                print("Finished last measurement. Set speed to 0")
            target_speed = 0

            circle_cords, radius = calculate_circle(measurement1,measurement2,measurement3)
            #print(calculate_circle(measurement1,measurement2,measurement3))

            print(circle_cords)
            print(radius)

            #angle = np.arctan(axe_distance / radius)
            #print("winkel: " )
            #print(angle)

            vec1 = (measurement1[0]-circle_cords[0],measurement1[1]-circle_cords[1])
            vec2 = (measurement3[0]-circle_cords[0],measurement3[1]-circle_cords[1])

            #dot = (vec1[0]*vec2[0]) + (vec1[1] * vec2[1])
            #det = (vec1[0]*vec2[0]) - (vec1[1] * vec2[1])

            dot = np.dot(vec1,vec2)
            norm = np.linalg.norm(vec1) * np.linalg.norm(vec2)



            #angle = np.arctan2(det, dot)

            angle = np.arccos(dot/norm)

            distance = angle * radius

            print("Winkel: ")
            print(angle)

            print("Distanz: ")
            print(distance)

            luftlinie = (measurement3[0]-measurement1[0], measurement3[1]-measurement1[1])

            print("Tick :")
            print(tick)

            print("meter/tickers: ")
            print(distance/tick)

            print("Luftlinie: ")
            print(np.linalg.norm(luftlinie))

            start = False
    
    #print(target_speed)       


    #print(target_speed)    
def calculate_circle(p1, p2, p3):
    x1 = p1[0]
    x2 = p2[0]
    x3 = p3[0]

    y1 = p1[1]
    y2 = p2[1]
    y3 = p3[1]

    A_mat = np.array([[x1,y1,1.0],[x2,y2,1.0],[x3,y3,1.0]])
    B_mat = np.array([[(x1**2 + y1**2) , y1, 1.0],[(x2**2 + y2**2) , y2, 1.0],[(x3**2 + y3**2) , y3, 1.0]])
    C_mat = np.array([[(x1**2 + y1**2) , x1, 1.0],[(x2**2 + y2**2) , x2, 1.0],[(x3**2 + y3**2) , x3, 1.0]])
    D_mat = np.array([[(x1**2 + y1**2) , x1, y1],[(x2**2 + y2**2) , x2, y2],[(x3**2 + y3**2) , x3, y3]])

    A = np.linalg.det(A_mat)
    B = -(np.linalg.det(B_mat))
    C = np.linalg.det(C_mat)
    D = -(np.linalg.det(D_mat))

    circle_x = -(B/(2*A))
    circle_y = -(C/(2*A))

    r = np.sqrt( ((B**2) + (C**2) - (4*A*D))/ (4 * (A**2))) 

    return (circle_x,circle_y), r






def threshold_publisher():
    global target_speed
    rospy.init_node('steering_calibration', anonymous=True)

    str_pub = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    speed_pub = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)

    rospy.Subscriber('/communication/gps/15', Odometry, callback)

    rospy.Subscriber('/sensors/arduino/ticks', Tick, tick_callback)

    rate = rospy.Rate(50)#hz

    seconds_passed = 0

    

    while not rospy.is_shutdown():

        str_msg = NormalizedSteeringCommand()
        str_msg.value = 1.0

        str_pub.publish(str_msg) 

        speed_msg = SpeedCommand()
        speed_msg.value = target_speed
        speed_pub.publish(speed_msg)
       

        seconds_passed += 0.02
        control_handler(seconds_passed, speed_pub)

        rate.sleep()

    

if __name__ == '__main__':
    try:
        threshold_publisher()
    except rospy.ROSInterruptException:
        pass