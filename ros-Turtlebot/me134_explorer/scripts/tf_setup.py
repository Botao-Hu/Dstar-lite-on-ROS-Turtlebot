#!/usr/bin/env python

# ME133B - Lab 3

# Execute this file to connect the turtlebot reference frames with its base

import rospy
import tf
from geometry_msgs.msg import Twist
import time
import math
import csv
 
if __name__ == '__main__':
    rospy.init_node('turtlebot_tf')
    br_rgbd = tf.TransformBroadcaster()
    br_lidar = tf.TransformBroadcaster()
    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        br_rgbd.sendTransform((0.0, 0.0, 0.18), (0.5, 0.5, 0.5, 0.5), rospy.Time.now(), "rgbd", "base_link")
        br_lidar.sendTransform((0.06, 0.0, 0.23), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "laser", "base_link")
        rate.sleep()
        