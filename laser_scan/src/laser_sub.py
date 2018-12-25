#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math

def laserCallback(scan_data):
    print("Listening to turtlebot")
    min_value, min_value_index = min_range_index(scan_data.ranges)
    print("\nmin value: {}".format(min_value))
    print("\nmin value index: {}".format(min_value_index))

    max_value, max_value_index = max_range_index(scan_data.ranges)
    print("\nmax value: {}".format(max_value))
    print("\nmax value index: {}".format(max_value_index))

    average_value = average_range(scan_data.ranges)
    print("\naverage value: {}".format(average_value))

    print("\nField of view : {}".format(field_of_view(scan_data)))

def min_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return (min(ranges), ranges.index(min(ranges)))

def max_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return (max(ranges), ranges.index(max(ranges)))

def average_range(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return (sum(ranges)/float(len(ranges)))

def field_of_view(scan_data):
    return (scan_data.angle_max - scan_data.angle_min)


if __name__=='__main__':
    rospy.init_node('Laser_scan_node')
    sub = rospy.Subscriber('/scan', LaserScan, laserCallback)
    rospy.spin()