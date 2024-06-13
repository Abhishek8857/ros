#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    closest_range = min(msg.ranges)
    print("Closest range: %0.1f" % closest_range)

rospy.init_node('closest_range')
closest_range_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()
