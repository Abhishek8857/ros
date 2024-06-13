#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

# Initialize the ROS node
rospy.init_node('red_light_green_light')

# Create a publisher to the 'cmd_vel' topic
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Define Twist messages for red light (stop) and green light (move)
red_light_twist = Twist()
green_light_twist = Twist()
green_light_twist.linear.x = 0.5

# Initialize state variables
driving_forward = False
light_change_time = rospy.Time.now() + rospy.Duration(3)

# Set the loop rate
rate = rospy.Rate(10)

# Main loop
while not rospy.is_shutdown():
    # Publish the appropriate Twist message based on the current state
    if driving_forward:
        cmd_vel_pub.publish(green_light_twist)
    else:
        cmd_vel_pub.publish(red_light_twist)

    # Check if it's time to change the light
    if light_change_time < rospy.Time.now():
        driving_forward = not driving_forward
        light_change_time = rospy.Time.now() + rospy.Duration(3)

    # Sleep to maintain the loop rate
    rate.sleep()
