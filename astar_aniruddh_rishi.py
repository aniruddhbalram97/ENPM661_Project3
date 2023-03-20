# import library
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
from queue import PriorityQueue

############# Global Variables #########
open_list = PriorityQueue() # unvisited list
#close_list = [] # visited list
close_list = np.zeros((500, 1200, 12), np.uint8)
visual_list = []
# using maps to store costs and parent nodes
cost_of_node = {} # map containing cost of each node
parent_node = {} # map containing parent node of each node
found_path = [] # list containing the found shortest path
#######################################

# input user data
def enter_coordinates(obstacle_map):
    x_i = int(input("Enter initial x-coordinate"))
    y_i = int(input("Enter initial y-coordinate"))
    x_g = int(input("Enter goal x-coordinate"))
    y_g = int(input("Enter goal y-coordinate"))
    orientation_s = int(input("Enter start orientation "))
    orientation_g = int(input("Enter goal orientation "))
    i = (x_i, obstacle_map.shape[0] - y_i - 1, orientation_s)
    g = (x_g, obstacle_map.shape[0] - y_g - 1, orientation_g)
    return i, g

# check for valid coordinates
def check_valid_entry(start, goal, obstacle_map):
    if(obstacle_map[start[1] + 5][start[0] + 5][0]==255 or obstacle_map[goal[1]+ 5][goal[0] + 5][0]==255 or 
       obstacle_map[start[1] - 5][start[0] - 5][0]==255 or obstacle_map[goal[1]- 5][goal[0] - 5][0]==255 or 
      (start[0] - 5) < 0 or (start[0] + 5) > 599 or (start[1] - 5) < 0 or (start[1] + 5) > 249 or sum(obstacle_map[start[1] + 5][start[0] + 5])==765
       or sum(obstacle_map[goal[1]][goal[0]])==765 or sum(obstacle_map[start[1] - 5][start[0] - 5])==765
       or sum(obstacle_map[goal[1] - 5][goal[0] - 5])==765
      ):
        print("The start point or end point is invalid. Either it is out of bounds or within obstacle space. Try Again!\n")
        return False
    else:
        print("Great! The start and goal points are valid")
        return True

