#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import time
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import TransformStamped
#from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# TODO: tf is deprecated since Hydro; use tf2

#from std_msgs.msg import Int8MultiArray

class ME134_Explorer:
    def __init__(self):
        self.last_goal = None
        self.last_map = None
        self.last_scan = None
        self.last_pose = None # this will be a tuple of (x position, y position, yaw angle)
        self.last_goal_status = None
        self.last_goal_accepted = False
        self.last_goal_reached = False
        self.last_goal_stamp = None
        self.abort = False
        self.mode = None

        self.goal_queue = []
        # Subscribe to all the necessary topics to get messages as they come in.
        rospy.init_node('me134_explorer', anonymous=False)
        #rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.poseCallback)
        rospy.Subscriber("map", OccupancyGrid, self.mapCallback)
        rospy.Subscriber("scan", LaserScan, self.scanCallback)
        rospy.Subscriber("move_base/status",GoalStatusArray, self.goalStatusCallback)
        rospy.loginfo('Subscribed to map, scan, move_base/status topics')
        
        # Create a transform buffer (buffers messages for up to 10 seconds), and a listener to recieve tf2 messages from it. 
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        #self.tfListener.callback(self.poseCallback) # this line doesn't work

        # all_frames_as_string is useful for debugging, to see what transforms exist. It needs the sleep call because it takes a moment to start
        #rospy.Rate(10).sleep()
        #rospy.loginfo('frames:'+ self.tfBuffer.all_frames_as_string())

        self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.loginfo('explorer online')
        pass
            

    def CheckIfHaveMapScanPose(self):
        return self.last_map and self.last_scan and self.last_pose

    def AddInplaceRotationsToQueue(self):
        import math
        import numpy
        last_x_y_yaw = (0,0,0)
        x,y,yaw = last_x_y_yaw
        for yaw_target in numpy.linspace(yaw,yaw+2*math.pi,10):
            self.goal_queue.append((x,y,yaw_target))
            pass
        pass
    
    def goalStatusCallback(self,goalStatusArray):
        """GoalID goal_id
        uint8 status
        #Allow for the user to associate a string with GoalStatus for debugging
        string text

        """
        
        if goalStatusArray.status_list:
            #rospy.loginfo(rospy.get_caller_id()+" goalStatusArray received: {}".format(goalStatusArray.status_list))
            pass
        self.last_goal_status = goalStatusArray
        for goal_status in goalStatusArray.status_list:
            #TODO verify that the goal_id matches the goal that we set
            goal_id = goal_status.goal_id.id
            goal_stamp = goal_status.goal_id.stamp
            goal_status_code = goal_status.status
            goal_status_text = goal_status.text
            if goal_stamp != self.last_goal_stamp:
                print "Ignoring old message:",goal_id,goal_status_text
                pass
            print "Goal has status: {}".format(goal_status_text)
            if goal_status_code == goal_status.PENDING:
                # The goal has yet to be processed by the action server
                pass
            elif goal_status_code == goal_status.ACTIVE:
                # The goal is currently being processed by the action server
                pass
            elif goal_status_code == goal_status.PREEMPTED:
                # The goal received a cancel request after it started executing
                #   and has since completed its execution (Terminal State)
                pass
            elif goal_status_code == goal_status.SUCCEEDED:
                # The goal was achieved successfully by the action server (Terminal State)
                self.last_goal_reached = True
                pass
            elif goal_status_code == goal_status.ABORTED:
                # The goal was aborted during execution by the action server due
                #    to some failure (Terminal State)
                self.abort = True
                pass
            elif goal_status_code == goal_status.REJECTED:
                # The goal was rejected by the action server without being processed,
                #    because the goal was unattainable or invalid (Terminal State)
                pass
            elif goal_status_code == goal_status.PREEMPTING:
                # The goal received a cancel request after it started executing
                #    and has not yet completed execution
                pass
            elif goal_status_code == goal_status.RECALLING:
                # The goal received a cancel request before it started executing,
                #    but the action server has not yet confirmed that the goal is canceled
                pass
            elif goal_status_code == goal_status.RECALLED:
                # The goal received a cancel request before it started executing
                pass
            pass
        #        [goal_id: 
        #  stamp: 
        #    secs: 25
        #    nsecs: 400000000
        #  id: "/move_base-1-25.400000000"
        #status: 3
        #text: "Goal reached."
        pass

    def getLastPose(self):
        try:
            # note: the 4th argument below means this call will block until the transform becomes available, or until timeout
            tfmsg = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
            # tfmsg is of type geometry_msgs/TransformStamped: http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html
            rospy.loginfo('map to base_link transform: '+ str(tfmsg))
            # parse info from TransformStamped message
            header = tfmsg.header
            translation = tfmsg.transform.translation
            orientation = tfmsg.transform.rotation
            
            # Create PoseStamped message from tfmsg. We are assuming here that map frame is at (0,0)                
            #position = Point(translation.x,translation.y,translation.z)
            #PSmsg = PoseStamped(header, Pose(position, orientation))
            #rospy.loginfo('PoseStamped message: '+str(self.last_pose))

            [pitch,roll,yaw] = tf.euler_from_quaternion(orientation)
            self.last_pose = (translation.x, translation.y, yaw)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return false

         #rospy.loginfo(rospy.get_caller_id()+" pose received: {}".format(poseData))
         #self.last_pose = poseData
         #print("pose received: {}".format(poseData))
        pass

    def mapCallback(self,occupancyGridData):
        #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
        #rospy.loginfo(rospy.get_caller_id()+" map received.")
        self.last_map = occupancyGridData
        #mapdata.data = occupancyGridData.data
        '''
        if mapData does ont have any -1s (no frontiers exist):
        mapExpored = True
        '''

        # also update self.last_pose from most recent transform 

        pass
    
    def scanCallback(self,scanData):
        #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
        #rospy.loginfo(rospy.get_caller_id()+" scan received.")
        self.last_scan = scanData
        pass
    
    def PublishGoal(self,x,y,yaw):
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
        self.last_goal = goal
        self.last_goal_stamp = goal.header.stamp
        rospy.loginfo(rospy.get_caller_id()+" set goal pose stamp: {}".format(goal.header.stamp))
        rospy.loginfo(rospy.get_caller_id()+" set goal pose: {}".format(goal.pose))
        self.last_goal_reached=False
        self.pub_goal.publish(goal)
        pass

    def Step(self):
        
        print "Mode=",self.mode
        if self.CheckIfHaveMapScanPose():
            if self.mode is None: # in the beginning
                self.AddInplaceRotationsToQueue()
                self.mode = "Rotating"
                x,y,yaw = self.goal_queue.pop()
                self.PublishGoal(x,y,yaw)
                pass
            if self.goal_queue:
                if self.last_goal_reached:
                    x,y,yaw = self.goal_queue.pop()
                    self.PublishGoal(x,y,yaw)
                    pass
                pass
            else:
                if self.last_goal_reached:
                    return True
                pass
            pass
        else:
            print "NoMapScan"
            self.getLastPose()
            pass
        if self.abort:
            return True
        return False



       

    pass

def explorer():
    # send command to turn around, to get an initial map

    brain = ME134_Explorer()
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        done = brain.Step()
        if done:
            break
        rate.sleep()
        pass
    pass

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
