from os import close
import numpy as np
from heapq import heappop, heappush
import matplotlib.pyplot as plt
import sys
import copy
import math as m
import argparse
from utils import *
import time


# Class:Node represents (x,y) position and saves other parameters at that coordinate. Acts as building block for A*.

"""

    Class Members:
    
    x: x coordinate of the position
    y: y coordinate of the position
    pose: (x,y)
    g_value: Optimal cost from start point to (x,y).
    h_value: Heauristic value
    f_value: g + h
    parent: Parent Node of the current Node 
    
    
"""
class Node(object):
    def __init__(self, pose):
        self.pose = np.array(pose)
        self.x = pose[0]
        self.y = pose[1]
        self.g_value = float("inf")
        self.h_value = 0
        self.f_value = float("inf")
        self.parent = None  

    def __lt__(self, other):
        if self.f_value == other.f_value:
            return self.h_value < other.h_value
        return self.f_value < other.f_value

    def __eq__(self, other):
        return (self.pose == other.pose).all()

# class Node(object):
#     def __init__(self, pose):
#         self.pose = np.array(pose)
#         self.x


# Class:AStar is the main A star algorithm class. Since an object of A* is defined, any function/part of A* can be called any number of times.
"""

    Class Members:
    
    map_p: Permanent/static 2D numpy array map extracted from the txt file that represents the world.
    y_dim: y dimension of map
    x_dim: x dimension of map
    visited: dictionary of nodes which have already been visited during A* expansion.
    weight: Weight value for weighted/Anytime A*
    collision threshold: Value above which the coordinate in map will be considered as an obstacle.
    max_time: maximum time available for planning
    start_time: time of initialization.
    
"""


class AStar(object):
    def __init__(self, map, weight, collision_thresh, max_time = float("inf")):
        self.map_p = copy.deepcopy(map)
        self.y_dim = self.map_p.shape[0]
        self.x_dim =self.map_p.shape[1]
        self.visited = {}
        self.weight = weight
        self.collision_threshold = collision_thresh
        self.max_time = max_time
        self.start_time = time.time()
        
    def getmapindex(self, x, y):
        return ((y)*(self.y_dim) + (x))
        
    def reset_map(self):
        self.visited = {}
        
    def heuristic(self, current, goal):
        dx = abs(goal.x - current.x)
        dy = abs(goal.y - current.y)
        h = (2**0.5)*min(dx,dy) + ((max(dx,dy))-(min(dx,dy)))
        return self.weight*h
        # return 0
        
    def get_successor(self, node):
        successor_list = []
        x,y = node.pose
        pose_list = [[x+1, y+1], [x, y+1], [x-1, y+1], [x-1, y],[x-1, y-1], [x, y-1], [x+1, y-1], [x+1, y]]

        for pose_ in pose_list:
            x_, y_ = pose_
            
            if 0 <= x_ < self.y_dim and 0 <= y_ < self.x_dim and self.map_p[x_,y_] < self.collision_threshold:
                successor_list.append(Node(pose_))
        
        return successor_list
    
    def calculate_path(self, node):
        path_ind = []
        path_ind.append(node.pose.tolist())
        current = node
        while current.parent is not None:
            current = current.parent
            path_ind.append(current.pose.tolist())
        path_ind.reverse()
        path = list(path_ind)

        return path

    # Main 2D A* planner algorithm
    def plan(self, start_ind, goal_ind):
        start_node = Node(start_ind)
        goal_node = Node(goal_ind)
        
        start_node.h_value= self.heuristic(start_node,goal_node)
        start_node.g_value = 0
        start_node.f_value = start_node.g_value+start_node.h_value
        self.reset_map()

        open_list = []
        closed_list = []
        heappush(open_list, start_node)

        while (len(open_list) > 0 and (time.time() - self.start_time) < self.max_time):
            current =heappop(open_list)
            closed_list = np.append(closed_list, current)

            if current == goal_node:
                return self.calculate_path(current)
            
            for successor in self.get_successor(current):
                if self.getmapindex(successor.x, successor.y) in self.visited:
                        successor = self.visited[self.getmapindex(successor.x, successor.y)]    
                        
                if successor not in closed_list:  
                    if ((successor.g_value) > current.g_value + self.map_p[successor.x,successor.y]):
                        successor.parent = current
                        successor.g_value = current.g_value + self.map_p[successor.x,successor.y] 
                        successor.h_value = self.heuristic(successor,goal_node)
                        successor.f_value = successor.g_value+successor.h_value
                        self.visited[self.getmapindex(successor.x, successor.y)] = successor
                        if (successor not in open_list): 
                            heappush(open_list, successor)

        # print('path not found')
        return None

    def run(self, cost_map, start_ind, goal_ind):
        if cost_map[start_ind[0], start_ind[1]] < self.collision_threshold and cost_map[goal_ind[0], goal_ind[1]] < self.collision_threshold:
            return self.plan(start_ind, goal_ind)

        else:
            print('Goal/Start position is in obstacle zone')




if __name__ == "__main__":
    start_time = time.time()
    parser = argparse.ArgumentParser()
    parser.add_argument("--txt_path", type=str, default="office_map.txt")
    parser.add_argument("--x_start", type=int)
    parser.add_argument("--x_end", type=int)
    parser.add_argument("--y_start", type=int)
    parser.add_argument("--y_end", type=int)
    parser.add_argument("--weight", type=int, default= 1)
    parser.add_argument("--collision_thresh", type=int)
    parser.add_argument("--max_time", type=int, default = 5)
    
    args = parser.parse_args()
    weight = args.weight
    collision_threshold = args.collision_thresh
    x_start = args.x_start
    y_start = args.y_start
    x_end = args.x_end
    y_end = args.y_end
    
    map = read_txt(args.txt_path)
    # planner = AStar(copy.deepcopy(map), weight=weight, collision_thresh= collision_threshold)
    # cost_map = copy.deepcopy(map)
    # path_list = planner.run(cost_map, [x_start, y_start], [x_end, y_end])
    # path_length = len(path_list)
    # print("Length of Path is {}".format(path_length))
    # print((time.time() - start_time))
    # visualize_path(map,path_list)
    
    final_path = None
    final_path_len = float("inf")
    planner = AStar(copy.deepcopy(map), weight, collision_threshold, max_time= args.max_time - (time.time() - start_time))
    cost_map = copy.deepcopy(map)
    while(time.time() - start_time < args.max_time):
        path_list = planner.run(cost_map, [x_start, y_start], [x_end, y_end])
        
        if path_list == None:
            break
        path_length = len(path_list)
        if final_path == None:
            final_path = path_list
        elif final_path_len > path_length:
            final_path_len = path_length
        planner.weight -=  weight/5
        if planner.weight < 1:
            planner.weight +=  weight/5
            break
        planner.max_time -= (time.time() - start_time)
        
    print("Planning time = {}".format(time.time() - start_time))
    print("Weight used for planning: {}".format(planner.weight))
    if final_path_len == float("inf"):
        print("Path not found, please enter proper coordinates")
    else:
        map = visualize_path(map,final_path)
        save_path(map)