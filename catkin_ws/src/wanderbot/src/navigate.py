#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

g_range_ahead, g_range_left, g_range_right = 1.0, 1.0, 1.0
cur_pos = None
cur_ort = None

def scan_callback(msg):
    global g_range_ahead, g_range_left, g_range_right
    left_side = min(msg.ranges[0:20])
    right_side = min(msg.ranges[340:360])   
    
    g_range_ahead = min(left_side, right_side)
    g_range_left = min(msg.ranges[70:110])
    g_range_right = min(msg.ranges[250:290])


def odom_callback(msg):
    global cur_pos, cur_ort
    cur_pos = msg.pose.pose.position
    cur_ort = msg.pose.pose.orientation
    
    
def stop_robot():
    twist = Twist()
    twist.linear.x = 0.6
    while twist.linear.x > 0.0:
        twist.linear.x -= 0.005
        if twist.linear.x < 0.0:
            twist.linear.x = 0.0
        cmd_vel_pub.publish(twist)
    rospy.loginfo("Stopping the Robot")


def forward():
    twist = Twist()
    twist.linear.x = 0.0
    while twist.linear.x < 0.4:
        twist.linear.x += 0.005
        if twist.linear.x > 0.4:
            twist.linear.x = 0.4
    cmd_vel_pub.publish(twist)
    
    
def turn_left():
    twist = Twist()
    twist.angular.z = 0.5
    cmd_vel_pub.publish(twist)
    
    
def turn_right():
    twist = Twist()
    twist.angular.z = -0.5
    cmd_vel_pub.publish(twist)
    
    

rospy.init_node('roam')

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
odom_sub =  rospy.Subscriber('odom', Odometry,  odom_callback)

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

state_change_time = rospy.Time.now()
random_turn_time = rospy.Time.now() + rospy.Duration(10)
rate = rospy.Rate(10)

rospy.on_shutdown(stop_robot)

state = "FORWARD"
TURN_DURATION = (np.pi)
print(TURN_DURATION)
log_count = 0
log_interval = 15

while not rospy.is_shutdown():
    
    log_count += 1
    
    if log_count > log_interval:
        rospy.loginfo(f"Current range ahead: {g_range_ahead}")
        rospy.loginfo(f"Current range left: {g_range_left}")
        rospy.loginfo(f"Current range right: {g_range_right}")
        log_count = 0


    if state == "FORWARD":
        rospy.loginfo("Moving Forward")
        if rospy.Time.now() > state_change_time:
            forward()

        if g_range_ahead < 0.7:
            state = "STOP"
            stop_robot()
            rospy.loginfo("Stopping")
            state_change_time = rospy.Time.now() + rospy.Duration(2)
        
        
    elif state == "STOP":
        if rospy.Time.now() > state_change_time:
            if g_range_left > g_range_right:
                state = "TURN_LEFT"
                state_change_time = rospy.Time.now() + rospy.Duration(TURN_DURATION)
                rospy.loginfo("Turning left")
                turn_left()
            else:
                state = "TURN_RIGHT"
                state_change_time = rospy.Time.now() + rospy.Duration(TURN_DURATION)
                rospy.loginfo("Turning Right")
                turn_right()
        
        
    elif state == "TURN_LEFT":
        if rospy.Time.now() > state_change_time:
            state = "FORWARD"
            state_change_time = rospy.Time.now() + rospy.Duration(2)
            forward()
    
    
    elif state == "TURN_RIGHT":
        if rospy.Time.now() > state_change_time:
            state = "FORWARD"
            state_change_time = rospy.Time.now() + rospy.Duration(2)
            forward()
                            
    rate.sleep()        
    
        


