#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

min_distance = 9
velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

def laserCallback(scan_data):
    global min_distance
    min_value = min_range_index(scan_data.ranges)
    min_distance = min_value
sub = rospy.Subscriber('/scan', LaserScan, laserCallback)
def min_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return min(ranges)

def rotate():
    global min_distance
    velocity_message = Twist()
    loop_rate = rospy.Rate(10)
    velocity_message.linear.x=0
    velocity_message.linear.y=0
    velocity_message.linear.z=0
    velocity_message.angular.x=0
    velocity_message.angular.y=0
    velocity_message.angular.z=0.5
    velocity_publisher.publish(velocity_message)

def move(speed, isforward):
    global min_distance
    velocity_message = Twist()
    
    #min_distance=min_value
    loop_rate = rospy.Rate(10)
    print("Min Distance : {}".format(min_distance))
    while True:
        print("No obstacle detected")
        while(min_distance>0.7):
            print("turtlebot moves")
            velocity_message.linear.x=1
            velocity_message.angular.z=0
            velocity_publisher.publish(velocity_message)
            print("Min Distance : {}".format(min_distance))
            loop_rate.sleep()
        print("Obstacle Detected")
        while(min_distance<1.0):
            print("turtlebot rotates")
            velocity_message.linear.x=0
            velocity_message.angular.z=0.7
            velocity_publisher.publish(velocity_message)
            print("Min Distance : {}".format(min_distance))
            loop_rate.sleep()

    velocity_message.linear.x=0
    velocity_publisher.publish(velocity_message)

if __name__=='__main__':
    rospy.init_node('Obstacle_detector')
    move(1.0,1)
    time.sleep(2)
    rospy.spin()



