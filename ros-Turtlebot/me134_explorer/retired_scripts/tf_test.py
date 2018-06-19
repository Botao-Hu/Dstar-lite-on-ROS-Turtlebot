#!/usr/bin/env python
import rospy
from tf import TransformListener
import time

rospy.init_node('tf_tester', anonymous=True)
time.sleep(1)
tf_listener = TransformListener()

while True:
    frames=tf_listener.allFramesAsString()
    if frames:
        #1/0
        pass
    if tf_listener.frameExists("base_link"): #and tf_listener.frameExists("map"):
        t = tf_listener.getLatestCommonTime("odom", "map")
        position, quaternion = tf_listener.lookupTransform("odom", "map", t)
        print "odom->map",position, quaternion
        t = tf_listener.getLatestCommonTime("base_link", "odom")
        position, quaternion = tf_listener.lookupTransform("base_link", "odom", t)
        print "base_link->odom",position, quaternion
        position, quaternion = tf_listener.lookupTransform("base_link", "map", rospy.Time.now()) # Problems with time in the past
        print "base_link->map",position, quaternion
        break
#    time.sleep(1)
    
