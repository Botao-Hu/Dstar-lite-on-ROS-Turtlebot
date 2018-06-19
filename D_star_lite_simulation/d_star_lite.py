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
    
    def __init__(self, map, x_goal, y_goal, x_start, y_start):
        # initialize
        self.start = np.array([x_start, y_start])
        self.goal = np.array([x_goal, y_goal])
        self.k_m = 0
        self.rhs = np.ones((len(map), len(map[0]))) * np.inf
        self.g = self.rhs.copy()
        self.global_map = map
        self.sensed_map = np.zeros((len(map), len(map[0])))
        self.rhs[self.goal[0], self.goal[1]] = 0
        self.queue = []
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
        while len(self.queue) > 0 and heapq.nsmallest(1, self.queue)[0] < Element(self.start, *self.CalculateKey(self.start))                 or self.rhs[self.start[0], self.start[1]] != self.g[self.start[0], self.start[1]]:
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
        row = len(self.global_map)
        col = len(self.global_map[0])
        real_list = []
        for s in s_list:
            if s[0] >= 0 and s[0] < row and s[1] >= 0 and s[1] < col:
                real_list.append(s)
        return real_list
    
    # calculate cost between nodes
    def cost(self, u1, u2):
        if self.sensed_map[u1[0],u1[1]] == np.inf or self.sensed_map[u2[0],u2[1]] == np.inf:
            # print('inf!')
            return np.inf
        else:
            #return 2 * np.linalg.norm(u1 - u2)
            return self.h_estimate(u1, u2)
    
    # sense the surroundings and return their real-time value
    def sense(self, range_s):
        real_list = []
        row = len(self.global_map)
        col = len(self.global_map[0])
        for i in range(-range_s, range_s+1):
            for j in range(-range_s, range_s+1):
                if self.start[0] + i >= 0 and self.start[0] + i < row and self.start[1] + j >= 0 and self.start[1] + j < col:
                    if not (i == 0 and j == 0):
                        real_list.append(np.array([self.start[0]+i,self.start[1]+j]))
        return real_list
                
        
def Main(map, x_goal, y_goal, x_start, y_start):
    ds = Dstar_lite(map, x_goal, y_goal, x_start, y_start)
    last = ds.start
    last, curr_update = Scan(ds, last)
    path = [ds.start]
    sensed_map = [ds.sensed_map.copy()]
    updated_points = [curr_update.copy()]
    ds.ComputeShortestPath()
    count = 0
    while np.sum(np.abs(ds.start - ds.goal)) != 0 and count < 100000:
        count += 1
        print("curr_location:", ds.start)
        s_list = ds.succ(ds.start)
        min_s = np.inf
        for s in s_list:
            if ds.cost(ds.start, s) + ds.g[s[0],s[1]] < min_s:
                # print(ds.cost(ds.start, s), ds.g[s[0],s[1]])
                min_s = ds.cost(ds.start, s) + ds.g[s[0],s[1]]
                temp = s
        ds.start = temp.copy()
        path.append(ds.start)
        sensed_map.append(ds.sensed_map.copy())
        last, curr_update = Scan(ds, last)
        updated_points.append(curr_update)
    return path, sensed_map, updated_points

# update map information and replan
def Scan(ds, last):
    s_list = ds.sense(3)
    flag = True
    updated_points = []
    for s in s_list:
        if ds.sensed_map[s[0],s[1]] != ds.global_map[s[0],s[1]]:
            flag = False
            print('See a wall!')
            break
    if flag == False:
        ds.k_m += ds.h_estimate(last, ds.start)
        last = ds.start.copy()
        for s in s_list:
            if ds.sensed_map[s[0],s[1]] != ds.global_map[s[0],s[1]]:
                updated_points.append(s[0])
                updated_points.append(s[1])
                ds.sensed_map[s[0],s[1]] = ds.global_map[s[0],s[1]]
                ds.UpdateVertex(s)
        ds.ComputeShortestPath()
    return last, np.asarray(updated_points)

# randomly generate connected maze
def maze(width=81, height=51, complexity=.75, density =.75):
    # Only odd shapes
    shape = ((height//2)*2+1, (width//2)*2+1)
    # Adjust complexity and density relative to maze size
    complexity = int(complexity*(5*(shape[0]+shape[1])))
    density    = int(density*(shape[0]//2*shape[1]//2))
    # Build actual maze
    Z = np.zeros(shape, dtype=float)
    # Fill borders
    Z[0,:] = Z[-1,:] = 1
    Z[:,0] = Z[:,-1] = 1
    # Make isles
    for i in range(density):
        x, y = rnd(0,shape[1]//2)*2, rnd(0,shape[0]//2)*2
        Z[y,x] = 1
        for j in range(complexity):
            neighbours = []
            if x > 1:           neighbours.append( (y,x-2) )
            if x < shape[1]-2:  neighbours.append( (y,x+2) )
            if y > 1:           neighbours.append( (y-2,x) )
            if y < shape[0]-2:  neighbours.append( (y+2,x) )
            if len(neighbours):
                y_,x_ = neighbours[rnd(0,len(neighbours)-1)]
                if Z[y_,x_] == 0:
                    Z[y_,x_] = 1
                    Z[y_+(y-y_)//2, x_+(x-x_)//2] = 1
                    x, y = x_, y_
    return Z


Z = maze(width=100, height=100)
Z[Z == 1] = np.inf
print(Z)
map = Z.copy()
x_goal = 99
y_goal = 99
x_start = 1
y_start = 1
# ds = Dstar_lite(map, x_goal, y_goal, x_start, y_start)
path, sensed_map, updated_points = Main(map, x_goal, y_goal, x_start, y_start)
print(path)

# print into txt, which is used for MATLAB to visualize
file = open('map.txt', 'w')
for line in map:
    for number in line:
        if number > 1: 
            number = 1
        file.write(str(int(number)) + ' ') 
    file.write('\n')
file.close()
    
file = open('path.txt', 'w')
for p in path:
    file.write(str(p[0] + 1) + ' ' + str(p[1] + 1) + '\n')
file.close()

file = open('sensed_map.txt', 'a')
for map in sensed_map:
    for line in map:
        for number in line:
            if number > 1: 
                number = 1
            file.write(str(int(number)) + ' ') 
        file.write('\n')
file.close()

file = open('updated_points.txt', 'a')
for line in updated_points:
    for number in line:
        file.write(str(int(number)) + ' ') 
    file.write('\n')
file.close()