#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#from std_msgs.msg import Int8MultiArray

global global_data
global_data = {}
global last_pose
global last_map
global last_scan
global pub_goal
global last_goal_status
last_pose = None

DEBUG =False

def PublishGoal(x,y,yaw):
    global pub_goal
    goal = PoseStamped()
    goal.header.frame_id = "/map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.z = 0
    goal.pose.position.y = y
    goal.pose.position.x = x
    q = quaternion_from_euler(0,0,yaw)
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]
    rospy.loginfo(rospy.get_caller_id()+" set goal pose: {}".format(goal.pose))
    pub_goal.publish(goal)
    
def explorer():
    global pub_goal
    # send command to turn around, to get an initial map

    
    #global mapdata
    #mapdata = Int8MultiArray()

    # listen to geometry_msgs and nav_msgs topics
    rospy.init_node('me134_explorer', anonymous=False)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, poseCallback)
    rospy.Subscriber("map", OccupancyGrid, mapCallback)
    rospy.Subscriber("scan", LaserScan, scanCallback)
    rospy.Subscriber("move_base/status",GoalStatusArray, goalStatusCallback)
    rospy.loginfo('explorer online')
    #rospy.loginfo(mapdata)
    # rospy.spin()

    pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        PublishGoal(3,3,0)
        rate.sleep()

    
'''
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)


    # TODO: look at spin(). we want the following to only occur once pose and map are obtained
    rospy.loginfo("Current pose andmap obtained. Finding frontier...")

    while not mapExplored:
        goalReached = False
        
        # find and publish next goal (i.e. which frontier to explore next)
        nextGoal = findGoal(poseData,mapData)
        rospy.loginfo(nextGoal)
        pub.publish(nextGoal)
        while not goalReached:
            wait
'''
def goalStatusCallback(goalStatusArray):
    if goalStatusArray.status_list:
        rospy.loginfo(rospy.get_caller_id()+" goalStatusArray received: {}".format(goalStatusArray.status_list))
        pass
    last_goal_status = goalStatusArray
    pass

def poseCallback(poseData):
    rospy.loginfo(rospy.get_caller_id()+" pose received: {}".format(poseData))
    last_pose = poseData
    print("pose received: {}".format(poseData))
    pass
    #if poseData == nextGoal: # TODO: make nextGoal variable accessible here
    #    goalReached = True

def mapCallback(occupancyGridData):
    #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
    #rospy.loginfo(rospy.get_caller_id()+" map received.")
    last_map = occupancyGridData
    #mapdata.data = occupancyGridData.data
'''
    if mapData does ont have any -1s (no frontiers exist):
        mapExpored = True
'''

def scanCallback(scanData):
    #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
    #rospy.loginfo(rospy.get_caller_id()+" scan received.")
    last_scan = scanData
          


def findGoal(poseData, mapData):
    nextGoal = PoseStamped()
    nextGoal.header.frame_id = "/map"
    nextGoal.header.stamp = rospy.Time.now()
    nextGoal.pose.position.z = 0.0
    nextGoal.pose.position.x = 1.0 #change this
    nextGoal.pose.position.y = 2.0 # change this
    nextGoal.pose.orientation.w = 1.0
    return nextGoal




if __name__ == '__main__':
    try:
        explorer()
    except rospy.ROSInterruptException:
        pass
