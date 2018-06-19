'''
D* lite simulation code
Botao Hu, Guanya Shi, Yukai Liu
CS133 Robotics Final Project
All rights reserved
'''

import numpy as np
import heapq
from numpy.random import random_integers as rnd
import matplotlib.pyplot as plt

'''
This class used to store data in priority queue.
Comparing methods are overloaded.
'''
class Element:
    def __init__(self, key, value1, value2):
        self.key = key
        self.value1 = value1
        self.value2 = value2

    def __eq__(self, other):
        return np.sum(np.abs(self.key - other.key)) == 0
    
    def __ne__(self, other):
        return self.key != other.key

    def __lt__(self, other):
        return ((self.value1, self.value2) < (other.value1, other.value2))

    def __le__(self, other):
        return ((self.value1, self.value2) <= (other.value1, other.value2))

    def __gt__(self, other):
        return ((self.value1, self.value2) > (other.value1, other.value2))

    def __ge__(self, other):
        return ((self.value1, self.value2) >= (other.value1, other.value2))

'''
Algorithm class
'''
class Dstar_lite:
    
    def __init__(self, map_width, map_height, x_goal, y_goal, x_start, y_start):
        # initialize
        self.start = np.array([x_start, y_start])
        self.goal = np.array([x_goal, y_goal])
        self.k_m = 0
        self.rhs = np.ones((map_width, map_height)) * np.inf
        self.g = self.rhs.copy()
        self.sensed_map = np.zeros((map_width, map_height))
        self.rhs[self.goal[0], self.goal[1]] = 0
        self.queue = []
        self.last = None
        A = Element(self.goal, *self.CalculateKey(self.goal))
        heapq.heappush(self.queue, A)
    
    def CalculateKey(self, s):
        key = [0, 0]
        # print(self.g[s[0],s[1]], self.rhs[s[0],s[1]], self.h_estimate(self.start, s), self.k_m)
        key[0] = min(self.g[s[0],s[1]], self.rhs[s[0],s[1]]) + self.h_estimate(self.start, s) + self.k_m
        key[1] = min(self.g[s[0],s[1]], self.rhs[s[0],s[1]])
        return key
    
    def UpdateVertex(self, u):
        if np.sum(np.abs(u - self.goal)) != 0:
            s_list = self.succ(u)
            min_s = np.inf
            for s in s_list:
                if self.cost(u, s) + self.g[s[0],s[1]] < min_s:
                    min_s = self.cost(u, s) + self.g[s[0],s[1]]
            self.rhs[u[0],u[1]] = min_s
        if Element(u, 0, 0) in self.queue:
            self.queue.remove(Element(u, 0, 0))
            heapq.heapify(self.queue)
        if self.g[u[0],u[1]] != self.rhs[u[0],u[1]]:
            heapq.heappush(self.queue, Element(u, *self.CalculateKey(u)))
            
    def ComputeShortestPath(self):
        while len(self.queue) > 0 and heapq.nsmallest(1, self.queue)[0] < Element(self.start, *self.CalculateKey(self.start))\
              or self.rhs[self.start[0], self.start[1]] != self.g[self.start[0], self.start[1]]:
            k_old = heapq.nsmallest(1, self.queue)[0]
            u = heapq.heappop(self.queue).key
            temp = Element(u, *self.CalculateKey(u))
            if k_old < temp:
                heapq.heappush(self.queue, temp)
            elif self.g[u[0],u[1]] > self.rhs[u[0],u[1]]:
                self.g[u[0],u[1]] = self.rhs[u[0],u[1]]
                s_list = self.succ(u)
                for s in s_list:
                    self.UpdateVertex(s)
            else:
                self.g[u[0],u[1]] = np.inf
                s_list = self.succ(u)
                s_list.append(u)
                for s in s_list:
                    self.UpdateVertex(s)
    
    # heuristic estimation
    def h_estimate(self, s1, s2):
        #return 0.0
        return np.linalg.norm(s1 - s2)
    
    # fetch successors and predessors
    def succ(self, u):
        s_list = [np.array([u[0]-1,u[1]-1]), np.array([u[0]-1,u[1]]), np.array([u[0]-1,u[1]+1]), np.array([u[0],u[1]-1]),                  np.array([u[0],u[1]+1]), np.array([u[0]+1,u[1]-1]), np.array([u[0]+1,u[1]]), np.array([u[0]+1,u[1]+1])];
        row = len(self.sensed_map)
        col = len(self.sensed_map[0])
        real_list = []
        for s in s_list:
            if s[0] >= 0 and s[0] < row and s[1] >= 0 and s[1] < col:
                real_list.append(s)
        return real_list
    
    # calculate cost between nodes
    def cost(self, u1, u2):
        if self.sensed_map[u1[0],u1[1]] == 100 or self.sensed_map[u2[0],u2[1]] == 100:
            # print('inf!')
            return np.inf
        else:
            #return 2 * np.linalg.norm(u1 - u2)
            return self.h_estimate(u1, u2)
    
    # sense the surroundings and return their real-time value
    def sense(self, range_s):
        real_list = []
        row = len(self.sensed_map)
        col = len(self.sensed_map[0])
        for i in range(-range_s, range_s+1):
            for j in range(-range_s, range_s+1):
                if self.start[0] + i >= 0 and self.start[0] + i < row and self.start[1] + j >= 0 and self.start[1] + j < col:
                    if not (i == 0 and j == 0):
                        real_list.append(np.array([self.start[0]+i,self.start[1]+j]))
        return real_list
                
        
def FindPath(map, x_start, y_start, ds, last_map_extents, last_pose, goal_queue):
    ds.start[0] = x_start
    ds.start[1] = y_start
    print("Start scanning")
    Scan(ds, map)
    fig2 = PlotSenseMap(ds, last_map_extents, last_pose, goal_queue)
    plt.show()

    # move starting point if it is in the wall
    if ds.dilated_map[ds.start[0], ds.start[1]] == 100:
        direction = FindClosestDirection(ds, map)
        ds.last = ds.start.copy()
        path = [ds.start, ds.start + 5*direction]
        return path

    ds.last = ds.start.copy()
    path = [ds.start]
    print("Start compute shortest path")
    ds.ComputeShortestPath()
    count = 0
    print("start while loop")
    last_move = np.array([0, 0])
    while np.sum(np.abs(ds.start - ds.goal)) != 0 and count < 100:
        count += 1
        print("curr_location:", ds.start)
        s_list = ds.succ(ds.start)
        min_s = np.inf
        for s in s_list:
            if ds.cost(ds.start, s) + ds.g[s[0],s[1]] < min_s and not np.array_equal((s - ds.start), -last_move):
                # print(ds.cost(ds.start, s), ds.g[s[0],s[1]])
                min_s = ds.cost(ds.start, s) + ds.g[s[0],s[1]]
                temp = s
                temp_last_move = s - ds.start
            #else:
            #    print(s, ":", ds.cost(ds.start, s), ds.g[s[0],s[1]], min_s)
        if temp is None or temp == ds.start:
            break
        last_move = temp_last_move.copy()    
        ds.start = temp.copy()
        path.append(ds.start)
    return path

# update map information and replan
def Scan(ds, map):
    if ds.last is not None:
        ds.k_m += ds.h_estimate(ds.last, ds.start)

    print(ds.sensed_map.shape, map.shape)

    ds.dilated_map = dilation(map, 10)

    diff_list = np.where(ds.sensed_map != ds.dilated_map)
    if len(diff_list) == 1:
        print("No update")
        return
    else:
        print("Update", diff_list[0].shape[0])
    for i in range(diff_list[0].shape[0]):
        ds.UpdateVertex(np.array([diff_list[0][i], diff_list[1][i]]))
    
    ds.sensed_map = np.copy(ds.dilated_map)

    return

def PlotSenseMap(ds, last_map_extents, last_pose, goal_queue):
    map_data = ds.sensed_map
    map_extents = last_map_extents
    
    fig,ax = plt.subplots()
    m = map_data
    image = np.zeros((m.shape[0],m.shape[1],3),dtype=np.uint8)
    # useful for debugging. Because numbers besides -1,0,or 100 won't show up on the map.
    #print set(m.flatten().tolist()) 
    image[m==-1] = (150,150,150)
    image[m==100] = (0,0,0)
    image[m==0] = (255,255,255)
    image[ds.goal[0],ds.goal[1]] = (255,0,0)
    image[ds.goal[0]-1,ds.goal[1]] = (255,0,0)
    image[ds.goal[0]+1,ds.goal[1]] = (255,0,0)
    image[ds.goal[0],ds.goal[1]-1] = (255,0,0)
    image[ds.goal[0],ds.goal[1]+1] = (255,0,0)
    
    ax.imshow(image,extent=map_extents,origin='lower',interpolation='none')
    ax.autoscale(tight=True)
    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")
    ax.set_title("map")
    x,y,yaw= last_pose
    ax.plot(x,y,'o', label="robot") # TODO: plot robot pointing direction 
    # ax.plot(ds.goal[0], ds.goal[1], '*', label="overall goal")
    if goal_queue: # If non-empty goal queue, plot first goal in the queue
        x,y,yaw= goal_queue[0]
        ax.plot(x,y,'x',label="next goal")
        pass
    fig.suptitle("sense map")
    ax.legend(loc='best', fancybox=True, framealpha=0.5)

    return fig

def dilation(map, dilation_size):
    new_map = np.zeros(map.shape)
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            if map[i, j] == 100:
                tmp_ix_str = i - dilation_size
                tmp_ix_end = i + dilation_size + 1
                tmp_iy_str = j - dilation_size
                tmp_iy_end = j + dilation_size + 1
                new_map[max(0, tmp_ix_str):min(map.shape[0], tmp_ix_end), max(0, tmp_iy_str):min(map.shape[1], tmp_iy_end)] = 100

    return new_map

def FindClosestDirection(ds, map):
    start_ix = ds.start[0]
    start_iy = ds.start[1]

    min_distance = np.inf
    min_direction = None
    for step in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        distance = 0
        current = [start_ix, start_iy]
        while map[current[0] + step[0], current[1] + step[1]] != 100 and distance < 20:
            current[0] += step[0]
            current[1] += step[1]
            distance += 1

        if distance < min_distance:
            min_distance = distance
            min_direction = step

    return -np.array(min_direction)