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