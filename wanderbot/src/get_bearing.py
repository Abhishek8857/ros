#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    for i in range(len(msg.ranges)):
        bearing = msg.angle_min + i * msg.angle_max/ len(msg.ranges)
        print("bearing: %0.1f" %bearing)

rospy.init_node('bearing')
bearing_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()
