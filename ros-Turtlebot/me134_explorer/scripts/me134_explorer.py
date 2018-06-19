#!/usr/bin/env python

'''
D* lite simulation code
Botao Hu, Guanya Shi, Yukai Liu, Yan Wu, Yu-Wei Wu
CS134 Autonomy Final Project
All rights reserved
'''

import rospy
import tf2_ros
import ros_numpy
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import LaserScan
#from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from std_msgs.msg import Int8MultiArray
import numpy
import math
import matplotlib.pyplot as plt
from d_star_lite import *


class ME134_Explorer:

    # Initialize ME134_Explorer 
    def __init__(self, strategy=None, safety_radius_m=None, initial_movement_m=None,
                 plot_global_costmap=0,
                 plot_map=1):
        self.strategy = strategy if strategy is not None else "FindDStarLite"
        self.safety_radius_m = safety_radius_m if safety_radius_m is not None else 0.4
        self.initial_movement_m = initial_movement_m if initial_movement_m is not None else -0.25
        self.plot_global_costmap = plot_global_costmap
        self.plot_map = plot_map
        self.last_goal = None

        self.last_map = None
        self.last_map_numpy = None
        self.last_map_extents = None
        
        self.last_global_costmap = None
        self.last_global_costmap_numpy = None
        self.last_global_costmap_extents = None

        self.last_pose = None # this will be a tuple of (x position, y position, yaw angle)
        self.abort = False
        self.mode = None
        self.next_mode = None
        self.have_first_data = False

        self.overlay = None

        #############
        #self.x_goal = 235
        #self.y_goal = 310
        self.x_goal = 400
        self.y_goal = 390
        self.path = None
        self.start_DStarLite = None
        self.last_DStarLite = None
        self.displacement_thre = 100

        self.goal_queue = []

        # Subscribe to all the necessary topics to get messages as they come in.
        rospy.init_node('me134_explorer', anonymous=False)
        #rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.poseCallback)
        rospy.Subscriber("map", OccupancyGrid, self.mapCallback)
        rospy.Subscriber("move_base/global_costmap/costmap", OccupancyGrid, self.globalCostMapCallback)
        #rospy.Subscriber("scan", LaserScan, self.scanCallback)
        rospy.loginfo('Subscribed to map, move_base/global_costmap/costmap topics')
        
        # Create a transform buffer (buffers messages for up to 10 seconds), and a listener to recieve tf2 messages from it. 
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # all_frames_as_string is useful for debugging, to see what transforms exist. 
        # It needs the sleep call because it takes a moment to start up
        #rospy.Rate(10).sleep()
        #rospy.loginfo('frames:'+ self.tfBuffer.all_frames_as_string())
        
        #self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.loginfo('explorer online')
        self.overlay_colors={1:(0,255,0),} # frontier color green
        pass

    def CheckIfHaveFirstData(self):
        return self.last_map and self.last_pose and self.last_global_costmap #and self.last_map_metadata 

    # This works, but you might find a better way
    def AddInplaceRotationsToQueue(self):
        x,y,yaw = self.last_pose
        for yaw_target in [math.pi/2,math.pi,-math.pi/2,0]:
            self.goal_queue.append((x,y,yaw_target))
            pass
        pass

    def AddMoveForwardToQueue(self,distance):
        x,y,yaw = self.last_pose
        x_new = x + distance*math.cos(yaw)
        y_new = y + distance*math.sin(yaw)
        self.goal_queue.append((x_new,y_new,yaw))
        pass

    
    def getLastPose(self):
        try:
            # Check the tf buffer for the latest transform
            # tfmsg is of type geometry_msgs/TransformStamped: http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html
            tfmsg = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            # Note: with a 4th argument, lookup_transform blocks until tranform recieved or until timeout
            #tfmsg = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
            
            # parse info from TransformStamped message
            #rospy.loginfo('map to base_link transform: '+ str(tfmsg))
            header = tfmsg.header
            trans = tfmsg.transform.translation
            orient = tfmsg.transform.rotation
            
            # Create PoseStamped message from tfmsg. We are assuming here that map frame is at (0,0)                
            #position = Point(translation.x,translation.y,translation.z)
            #PSmsg = PoseStamped(header, Pose(position, orientation))
            #rospy.loginfo('PoseStamped message: '+str(self.last_pose))

            # Note: apparently the geometry_msgs quaternion is not the same as the tf.transformations quaternion,
            # so we need to do this explicitly rather than simply using euler_from_quaternion(orientation)
            [pitch,roll,yaw] = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])
            self.last_pose = (trans.x, trans.y, yaw)
            rospy.loginfo('Pose: '+str(self.last_pose))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False
        return True
    

    def mapCallback(self,occupancyGridData):
        #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
        #rospy.loginfo(rospy.get_caller_id()+" map received.")
        self.last_map = occupancyGridData
        # occupancygrid_to_numpy returns a masked numpy array, we use the .data to remove the mask.
        self.last_map_numpy = ros_numpy.occupancy_grid.occupancygrid_to_numpy(occupancyGridData).data
        #self.last_map_from_above = numpy.flipud(self.last_map_numpy)
        
        resolution = occupancyGridData.info.resolution
        left = occupancyGridData.info.origin.position.x
        right = left + occupancyGridData.info.width*resolution
        bottom = occupancyGridData.info.origin.position.y
        top = bottom + occupancyGridData.info.height*resolution
        self.last_map_extents = (left,right,bottom,top) # for plotting
        print("last_map_extents", self.last_map_extents, resolution, occupancyGridData.info.width, occupancyGridData.info.height)

        pass

    def globalCostMapCallback(self,occupancyGridData):
        #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
        #rospy.loginfo(rospy.get_caller_id()+" map received.")
        self.last_global_costmap = occupancyGridData
        # occupancygrid_to_numpy returns a masked numpy array, we use the .data to remove the mask.
        self.last_global_costmap_numpy = ros_numpy.occupancy_grid.occupancygrid_to_numpy(occupancyGridData)
        #self.last_map_from_above = numpy.flipud(self.last_map_numpy)
        
        resolution = occupancyGridData.info.resolution
        left = occupancyGridData.info.origin.position.x
        right = left + occupancyGridData.info.width*resolution
        bottom = occupancyGridData.info.origin.position.y
        top = bottom + occupancyGridData.info.height*resolution
        self.last_global_costmap_extents = (left,right,bottom,top)
        print("last_global_costmap_extents", self.last_global_costmap_extents, resolution, occupancyGridData.info.width, occupancyGridData.info.height)
    
        pass
        
    def PlotMap(self,overlay=None,overlay_colors=None):
        assert self.last_map
        m = self.last_map_numpy
        map_extents = self.last_map_extents
        
        fig,ax = plt.subplots()
        image = numpy.zeros((m.shape[0],m.shape[1],3),dtype=numpy.uint8)
        # useful for debugging. Because numbers besides -1,0,or 100 won't show up on the map.
        #print set(m.flatten().tolist()) 
        image[m==-1] = (150,150,150)
        image[m==100] = (0,0,0)
        image[m==0] = (255,255,255)
        # you can set your own overlay colors
        if overlay is not None:
            assert overlay_colors
            for value,color in overlay_colors.iteritems():
                image[overlay==value] = color
                pass
            pass
        ax.imshow(image,extent=map_extents,origin='lower',interpolation='none')
        ax.autoscale(tight=True)
        ax.set_xlabel("x position (m)")
        ax.set_ylabel("y position (m)")
        ax.set_title("map")
        x,y,yaw= self.last_pose
        ax.plot(x,y,'o', label="robot") # TODO: plot robot pointing direction 
        if self.goal_queue: # If non-empty goal queue, plot first goal in the queue
            x,y,yaw= self.goal_queue[0]
            ax.plot(x,y,'x',label="next goal")
            pass
        fig.suptitle("mode={} Close plots to proceed".format(self.mode))
        ax.legend(loc='best', fancybox=True, framealpha=0.5)

        return fig

    # plots the most recent costmap (from subscribed topic)
    def PlotCostMap(self):
        assert self.last_global_costmap
                
        #plt.imshow(self.last_map_from_above,extent=self.last_map_from_above_extents)
        fig,ax = plt.subplots()
        m = self.last_global_costmap_numpy
        m_extents = self.last_global_costmap_extents
        ax.imshow(m,extent=m_extents,origin='lower',cmap='Oranges',interpolation='none')
        ax.autoscale(tight=True)
        ax.set_xlabel("x position (m)")
        ax.set_ylabel("y position (m)")
        ax.set_title("global cost map (sometimes doesn't update)")
        fig.suptitle("mode={} Close plots to proceed".format(self.mode))
        #x,y,yaw= self.Get_x_y_yaw()
        x,y,yaw= self.last_pose
        ax.plot(x,y,'o',label="robot") 
        if self.goal_queue:
            x,y,yaw= self.goal_queue[0]
            ax.plot(x,y,'x',label="next goal")
            pass
        ax.legend(loc='best', fancybox=True, framealpha=0.5)
        return fig

    # def scanCallback(self,scanData):
    #     #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
    #     #rospy.loginfo(rospy.get_caller_id()+" scan received.")
    #     self.last_scan = scanData
    #     pass
    
    def PublishGoal(self,x,y,yaw):
        self.last_goal = (x,y,yaw)
        rospy.loginfo(rospy.get_caller_id()+" requesting to move to {}".format((x,y,yaw)))
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0,0,yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Unfortunately this result doesn't seem to tell us if the goal was achieved.
            # Pull requests that actually get this information would be appreciated.
            return client.get_result()
        pass

    # returns indices of frontiers
    def FindFrontier(self,m):
        free_space_indices = numpy.where(m==0)
        #print "free_space_indices=",free_space_indices
        # Note: this is a brute force way - feel free to improve on it!
        frontier_indices = []
        for (ia,ib) in zip(*free_space_indices): # iterates through all free spaces
            has_unknown=False
            has_free=False
            # We've defined a frontier as a free space with at least 1 unknown and 1 free non-diagonal neighbor
            for off_ia,off_ib in ((-1,0),(1,0),(0,-1),(0,1)): 
                if has_unknown and has_free:
                    break
                ia_new,ib_new = ia+off_ia,ib+off_ib
                if 0<=ia_new<m.shape[0] and 0<=ib_new<m.shape[1]:
                    if m[ia_new,ib_new] == -1:
                        has_unknown=True
                        pass
                    if m[ia_new,ib_new] == 0:
                        has_free=True
                        pass
                    pass
                else:
                    print "WARNING: free space at map edge"
                    pass
                pass
            if has_unknown and has_free:
                frontier_indices.append((ia,ib))
                pass
            pass
        return frontier_indices

    def FindRandomEmptySpace(self, safety_radius_m=None):
        info = self.last_map.info
        resolution = info.resolution
        m = self.last_map_numpy
        #rx,ry,ryaw = self.last_pose
        #robot_cell = self.ConvertMetersToMapIndices(rx,ry)

        safety_radius_cells = safety_radius_m/resolution

        free_space_indices = numpy.where(m==0)
        assert free_space_indices
        permutation = numpy.random.permutation(len(free_space_indices[0]))
        for i in permutation:
            ia,ib = free_space_indices[0][i],free_space_indices[1][i]
            small_corner = (ia-safety_radius_cells,ib-safety_radius_cells)
            large_corner = (ia+safety_radius_cells,ib+safety_radius_cells)
            if small_corner[0] < 0 or small_corner[1] < 0:
                continue
            if large_corner[0] >= m.shape[0] or large_corner[1] >= m.shape[1]:
                continue
            square = m[small_corner[0]:large_corner[0],small_corner[1]:large_corner[1]]
            s = set(square.flatten().tolist())
            if -1 in s or 100 in s:
                # There are walls or unknown space in the square, path planning might fail
                continue
            # Target found
            tx,ty = self.ConvertMapIndicesToMeters(ia,ib)
            return tx,ty
        print "Failed to find any random safe place with safety_radius_m ={}".format(safety_radius_m)
        return None

    # find the closest point on a frontier that is at least min_required_distance away
    # NOTE: If you are writing your own exploration algorithm, you will probably want to replace this method with your own
    def FindClosestFrontier(self,min_required_distance=None):
        rx,ry,r_yaw = self.last_pose
        min_distance = None
        target_x_y = None
        # Brute force approach: convert all frontier indices to distances
        for ia,ib in self.frontier_indices:
            tx,ty = self.ConvertMapIndicesToMeters(ia,ib)
            distance = math.sqrt((rx-tx)**2+(ry-ty)**2)
            if min_required_distance is not None and distance < min_required_distance:
                print "Skipping point {} with distance {} which might be under the robot".format((tx,ty),distance)
                continue
            if min_distance is None or distance < min_distance:
                min_distance = distance
                target_x_y = (tx,ty)
                pass
            pass
        print "Closest Frontier is at {}. It is {} meters from the robot.".format(target_x_y,min_distance)
        return target_x_y

    def ConvertMapIndicesToMeters(self,ia,ib):
        info = self.last_map.info
        resolution = info.resolution
        x = info.origin.position.x + ib*resolution
        y = info.origin.position.y + ia*resolution
        return x,y

    def ConvertMetersToMapIndices(self,x,y):
        info = self.last_map.info
        resolution = info.resolution
        # This code hasn't been checked and might not round to the closest index
        ib = int((x - info.origin.position.x)/resolution)
        ia = int((y - info.origin.position.y)/resolution) 
        return ia,ib

    def map_filter(self, map):
        new_map = np.zeros(map.shape)
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i, j] == 100:
                    new_map[i, j] = 100

        return new_map

    # find the closest point on a frontier that is at least min_required_distance away
    # NOTE: If you are writing your own exploration algorithm, you will probably want to replace this method with your own
    def TestTest123(self):
        safe_distance = 7
        rx,ry,r_yaw = self.last_pose
        ix, iy = self.ConvertMetersToMapIndices(rx, ry)

        min_distance = None
        target_x_y = None
        farthest = 0
        for step in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            distance = 0
            current = [ix, iy]
            while True:
                flag = True
                for i in range(-safe_distance, safe_distance+1):
                    for j in range(-safe_distance, safe_distance+1):
                        if self.last_map_numpy[current[0] + i, current[1] + j] != 0:
                            flag = False
                            break
                    if not flag:
                        break
                if not flag:
                    break
                current[0] += step[0]
                current[1] += step[1]
                distance += 1

            if distance > farthest:
                farthest = distance
                target_x_y = (current[0] - step[0], current[1] - step[1])

        target_x_y = self.ConvertMapIndicesToMeters(*target_x_y)
        return target_x_y

    ######## DStarLite
    def DStarLite(self):
        if self.last_DStarLite is None:
            rx,ry,r_yaw = self.last_pose
            ix, iy = self.ConvertMetersToMapIndices(rx, ry)
            self.start_DStarLite = np.array([ix, iy])
        else:
            ix = self.last_DStarLite[0]
            iy = self.last_DStarLite[1]

        if self.path is None:
            new_map = self.map_filter(self.last_map_numpy)
            self.path = FindPath(new_map, ix, iy, self.ds, self.last_map_extents, self.last_pose, self.goal_queue)

        print("PATH:")
        print(self.path)

        displacement = None
        current = np.array([ix, iy])
        flag = False
        for i in range(1, len(self.path)):
            tmp = self.path[i]
            if displacement is None:
                displacement = tmp - current
                current = tmp
            elif np.array_equal((tmp - current), displacement) and \
                 np.linalg.norm(tmp-self.start_DStarLite) < self.displacement_thre and \
                 self.last_map_numpy[tmp[0], tmp[1]] != -1 and not np.array_equal(tmp, current):
                current = tmp
            else:
                flag = True
                break

        if flag:
            self.path = self.path[i-1:]
        else:
            self.path = self.path[i:]
        self.last_DStarLite = current

        print("Next goal: ", current)

        target_x_y = self.ConvertMapIndicesToMeters(current[0], current[1])
        target_ix_iy = np.array(current)
        return target_x_y, target_ix_iy

    '''
    elif np.array_equal((tmp - current), displacement) and \
         np.linalg.norm(tmp-self.start_DStarLite) < self.displacement_thre and \
         self.last_map_numpy[tmp[0], tmp[1]] != -1 and not np.array_equal(tmp, current):
        current = tmp
    '''

    # builds frontiers and makes overlay (to plot frontiers on top of map)
    def ProcessMap(self):
        self.frontier_indices = self.FindFrontier(self.last_map_numpy)
        self.overlay = numpy.zeros(self.last_map_numpy.shape)
        for (ia,ib) in self.frontier_indices:
            self.overlay[ia,ib]=1 # mark as frontier
            pass
        pass

    def Step(self):
        print "Mode=",self.mode
        plotEachStep=True
        self.getLastPose()

        if self.last_map_numpy is not None:
            print("Map shape: ", self.last_map.info.height, self.last_map.info.width)

        if self.CheckIfHaveFirstData():
            ######## DStarLite
            if not self.have_first_data and self.last_map_numpy.shape[0] == 544 and self.last_map_numpy.shape[1] == 544:
            #if not self.have_first_data:
                rx,ry,r_yaw = self.last_pose
                ix, iy = self.ConvertMetersToMapIndices(rx, ry)
                self.ds = Dstar_lite(544, 544, self.x_goal, self.y_goal, ix, iy)
                self.have_first_data = True

            if self.last_goal:
                distance_to_last_goal = math.sqrt((self.last_pose[0]-self.last_goal[0])**2+(self.last_pose[1]-self.last_goal[1])**2)
                if distance_to_last_goal > self.safety_radius_m:
                    print "Failed to achieve goal {} robot is at {}, distance={}".format(self.last_goal,self.last_pose,distance_to_last_goal)
                    pass
                pass
            if self.mode == "FindClosestFrontier":
                self.ProcessMap()
            print "goal_queue={}".format(self.goal_queue)
            if plotEachStep and (self.mode not in ["Rotating","Initial Movement"]): # self.mode - FSM markign current state
                if self.plot_global_costmap:
                    fig1=self.PlotCostMap()
                    pass
                if self.plot_map:
                    fig2=self.PlotMap(overlay=self.overlay,overlay_colors=self.overlay_colors)
                    pass
                if self.plot_map or self.plot_global_costmap:
                    plt.show()  # have to close windows for program to continue
                    pass
                if self.plot_global_costmap:
                    plt.close(fig1) # this frees up the figures so as to not run out of memory
                    pass
                if self.plot_map:
                    plt.close(fig2)
                    pass
                pass
            if self.goal_queue: # if there's anything in the goal queue, do it
                x,y,yaw = self.goal_queue.pop(0)
                goal_result = self.PublishGoal(x,y,yaw)
                # goal_result here appears to be always empty.
                #print "PublishGoal({},{},{}) returned {}".format(x,y,yaw,goal_result)
                pass
            else: # once done with goal queue, move on to the next step, given by self.next_mode
                # Change the following code to run your algorithm
                # *------------------------------------------------*
                self.mode = self.next_mode
                
                if self.mode is None: # in the beginning
                    #self.goal_queue.append((0,0,0))
                    # There is a bug https://answers.ros.org/question/204740/dwa_local_planner-in-place-rotation-first-goal-does-not-work/
                    # That means that the first movement can't be a turn in place
                    # If we reverse a bit first the robot will rotate to clear the costmap
                    # After that, you can move robot in any way you want
                    # TODO: maybe you can find a better way to do this

                    self.AddMoveForwardToQueue(self.initial_movement_m)
                    # Then return to the starting place
                    self.AddMoveForwardToQueue(-self.initial_movement_m)
                    self.mode = "Initial Movement"
                    self.next_mode = self.strategy
                    #self.next_mode = "Rotating" # If you still have visible frontiers 
                    pass
                elif self.mode == "FindClosestFrontier":
                    self.next_mode = "Rotating"
                    # You should find a position on the map that views a frontier
                    # from a safe distance using the global_cost_map and/or
                    # another algorithm. Warning: global costmap doesn't always update regularly. 
                    # You can probably fix this if you want by subscribing to global_costmap_updates. 
                    # I'm only going to attempt to go to the closest one (probably not safe) for the demo.
                    target_x_y = self.FindClosestFrontier(min_required_distance=self.safety_radius_m) 
                    # Min distance because otherwise robot will try to go to the frontier directly under the robot - at least at first, the laser scanner won't scan that!
                    print("WARNING: Unless you change this code it won't drive anywhere. The goal is too close to a wall. You need to find a place to safely view this spot.")
                    
                    if target_x_y is not None:
                        tx,ty=target_x_y
                        # we're setting yaw=0 here. You may want to compute yaw angle, or choose it carefully...
                        self.goal_queue.append((tx,ty,0))
                        pass
                    else:
                        print "No Frontiers found"
                        self.abort = True
                        pass
                elif self.mode == "FindRandomEmptySpace":
                    self.next_mode = "Rotating"
                    # This just finds a random empty square of size 2*self.safety_radius_m by  2*self.safety_radius_m and attempts to go there.
                    # This is a brute force random "dumb" way to explore. Students are expected to do something more efficient than this.
                    target_x_y = self.FindRandomEmptySpace(safety_radius_m=self.safety_radius_m)

                    if target_x_y is not None:
                        tx,ty=target_x_y
                        # we're setting yaw=0 here. You may want to compute yaw angle, or choose it carefully...
                        self.goal_queue.append((tx,ty,0))
                        pass
                    else:
                        print "No Random free space found"
                        self.abort = True
                        pass
                elif self.mode == "FindTestTest123":
                    self.next_mode = "Rotating"
                    # This just finds a random empty square of size 2*self.safety_radius_m by  2*self.safety_radius_m and attempts to go there.
                    # This is a brute force random "dumb" way to explore. Students are expected to do something more efficient than this.
                    target_x_y = self.TestTest123()

                    if target_x_y is not None:
                        tx,ty=target_x_y
                        # we're setting yaw=0 here. You may want to compute yaw angle, or choose it carefully...
                        self.goal_queue.append((tx,ty,0))
                        pass
                    else:
                        print "No TestTest123 free space found"
                        self.abort = True
                        pass
                elif self.mode == "FindDStarLite":
                    self.next_mode = "Rotating"
                    # This just finds a random empty square of size 2*self.safety_radius_m by  2*self.safety_radius_m and attempts to go there.
                    # This is a brute force random "dumb" way to explore. Students are expected to do something more efficient than this.
                    target_x_y, target_ix_iy = self.DStarLite()
                    count = 0
                    while count < 5 and \
                          np.linalg.norm(target_ix_iy-self.start_DStarLite) < self.displacement_thre and \
                          self.last_map_numpy[target_ix_iy[0], target_ix_iy[1]] != -1:
                        count += 1
                        tx,ty=target_x_y
                        # we're setting yaw=0 here. You may want to compute yaw angle, or choose it carefully...
                        self.goal_queue.append((tx,ty,0))
                        if target_ix_iy[0] == self.x_goal and target_ix_iy[1] == self.y_goal:
                            print("reach goal")
                            self.abort = True
                            break
                        if len(self.path) == 1:
                            break
                        target_x_y, target_ix_iy = self.DStarLite()
                    self.path = None
                    self.start_DStarLite = None
                    self.last_DStarLite = None

                    pass

                elif self.mode == "Rotating":
                    self.AddInplaceRotationsToQueue()
                    self.next_mode = self.strategy
                    pass
                else:
                    print "Unknown mode={}, perhaps your strategy is not recognized".format(self.mode)
                    return True
                # *-------------------------------------------------------------*
                pass
            pass
        else:
            print "NoMap yet"
            pass
        if self.abort and not self.goal_queue:
            return True
        return False
    pass

def explorer():
    import argparse
    safety_radius_m_default = 0.25 # 0.1 doesn't work, 0.4 works
    initial_movement_m_default = -0.25
    plot_global_costmap_default=0
    plot_map_default=1


    parser = argparse.ArgumentParser()
    parser.add_argument('--strategy', choices=['FindClosestFrontier', 'FindRandomEmptySpace', 'FindTestTest123', 'FindDStarLite'], default='FindDStarLite')
    parser.add_argument('--safety_radius_m', type=float, default=safety_radius_m_default, help="default: {}".format(safety_radius_m_default))
    parser.add_argument('--initial_movement_m', type=float, default=initial_movement_m_default, help="default: {}".format(initial_movement_m_default))

    parser.add_argument('--plot_global_costmap', type=int, default=plot_global_costmap_default, help="default: {}".format(plot_global_costmap_default))
    parser.add_argument('--plot_map', type=int, default=plot_map_default, help="default: {}".format(plot_map_default))

    args = parser.parse_args()

    brain = ME134_Explorer(strategy=args.strategy,
                           safety_radius_m=args.safety_radius_m,
                           initial_movement_m=args.initial_movement_m,
                           plot_map=args.plot_map,
                           plot_global_costmap=args.plot_global_costmap)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        done = brain.Step()
        if done:
            break
        rate.sleep()
        pass
    pass


if __name__ == '__main__':
    try:
        explorer()
    except rospy.ROSInterruptException:
        pass
