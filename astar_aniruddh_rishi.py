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

# Function to generate map using half plane equations
def generate_map(canvas):
    # Points for hexagon
    a = 75
    h = int((math.sqrt(3)/2) * a)
    # Hexagon with clearance
    x1, y1 = 300 - h - 5, int(125 - a/2 - 2.5)
    x2 ,y2 = 300, int(125 - a - 5)
    m1 = (y2 - y1)/(x2 - x1) # slope of left top line
    x3, y3 = 300 - h - 5, int(125 + a/2 + 2.5)
    x4 ,y4 = 300, int(125 + a + 5)
    m2 = (y4 - y3)/(x4 - x3)
    x5, y5 = 300 + h + 5, int(125 + a/2 + 2.5)
    m3 = (y5 - y4)/(x5 - x4)
    x6, y6 = 300 + h + 5, int(125 - a/2 - 2.5)
    m4 = (y6 - y2)/(x6 - x2)
    
    # Hexagon without clearance
    x1_, y1_ = 300 - h, int(125 - a/2)
    x2_ ,y2_ = 300, int(125 - a)
    m1_ = (y2_ - y1_)/(x2_ - x1_) # slope of left top line
    x3_, y3_ = 300 - h, int(125 + a/2)
    x4_ ,y4_ = 300, int(125 + a)
    m2_ = (y4_ - y3_)/(x4_ - x3_)
    x5_, y5_ = 300 + h, int(125 + a/2)
    m3_ = (y5_ - y4_)/(x5_ - x4_)
    x6_, y6_ = 300 + h, int(125 - a/2)
    m4_ = (y6_ - y2_)/(x6_ - x2_)
    
    # Triangle with clearance
    u1, v1 = 460 - 5, 25 - 5
    u2, v2 = 460 - 5, 225 + 5
    u3, v3 = 510 + 10, 125
    t1 = (v2 - v3)/(u2 - u3)
    t2 = (v1 - v3)/(u1 - u3)
    
    # Triangle without clearance
    u1_, v1_ = 460, 25 + 9.48
    u2_, v2_ = 460, 225 - 9.48
    u3_, v3_ = 510 , 125
    t1_ = (v2_ - v3_)/(u2_ - u3_)
    t2_ = (v1_ - v3_)/(u1_ - u3_)
    canvas_ = canvas.copy()
    for j in range(canvas.shape[0]):
        for i in range(canvas.shape[1]):
            # create rectangles with clearance.
            if(i>=95 and i < 155 and j<=105):
                canvas_[j,i] = [255, 0 ,0]
            if(i>=95 and i < 155 and j>=145):
                canvas_[j,i] = [255, 0 ,0]
                
            # create rectangles without clearance
            if(i>=100 and i < 150 and j<=100):
                canvas_[j,i] = [255, 255 ,255]
            if(i>=100 and i < 150 and j>=150):
                canvas_[j,i] = [255, 255 ,255]
            
            # create hexagon with clearance
            # divide the hexagon into 2 halves and compute line equations for each
            if(i>=(300 - h - 5) and i < 300 and ((j - m1*i - y1 + m1*x1)>= 0 and (j - m2*i - y3 + m2*x3)<= 0)):
                canvas_[j, i] = [255, 0, 0]
            if(i<(300 + h + 5) and i >= 300 and ((j - m3*i - y4 + m3*x4)<= 0 and (j - m4*i - y2 + m4*x2)>= 0)):
                canvas_[j, i] = [255, 0, 0]
                
            # create hexagon without clearance
            # divide the hexagon into 2 halves and compute line equations for each
            if(i>=(300 - h) and i < 300 and ((j - m1_*i - y1_ + m1_*x1_)>= 0 and (j - m2_*i - y3_ + m2_*x3_)<= 0)):
                canvas_[j, i] = [255, 255, 255]
            if(i<(300 + h) and i >= 300 and ((j - m3_*i - y4_ + m3_*x4_)<= 0 and (j - m4_*i - y2_+ m4_*x2_>= 0))):
                canvas_[j, i] = [255, 255, 255]
                
            # create triangle with clearance
            if(i>=(460 - 5) and ((j - t1*i - v3 + t1*u3)<= 0 and (j - t2*i - v3 + t2*u3)>= 0)):
                canvas_[j, i] = [255, 0, 0]
            # create triangle without clearance
            if(i>(460) and ((j - t1_*i - v3_ + t1_*u3_)<= 0 and (j - t2_*i - v3_ + t2_*u3_)>= 0)):
                canvas_[j, i] = [255, 255, 255]
            
            # clearance for walls
            if(i<5 or j<5 or i>594 or j>244):
                canvas_[j, i] = [255, 0 ,0]
            
    return canvas_

# Next moves
def move_neg_sixty(current_node, step_size):  
    next_node = []
    theta = -60
    next_x = int(round((current_node[0] + step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
    next_y = int(round((current_node[1] + step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
    next_orientation = theta + current_node[2]

    # For negative index in the visited matrix
    if(next_orientation < 0):
        next_orientation+=360
    next_orientation%=360
    
    next_node.append(next_x)
    next_node.append(next_y)
    next_node.append(next_orientation)
    return step_size, tuple(next_node)

# Next moves
def move_neg_thirty(current_node, step_size):  
    next_node = []
    theta = -30
    next_x = int(round((current_node[0] + step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
    next_y = int(round((current_node[1] + step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
    next_orientation = theta + current_node[2]
    
    # For negative index in the visited matrix
    if(next_orientation < 0):
        next_orientation+=360
    next_orientation%=360
    
    next_node.append(next_x)
    next_node.append(next_y)
    next_node.append(next_orientation)
    return step_size, tuple(next_node)

# Next moves
def move_zero(current_node, step_size):  
    next_node = []
    theta = 0
    next_x = int(round((current_node[0] + step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
    next_y = int(round((current_node[1] + step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
    next_orientation = theta + current_node[2]
    
    # For negative index in the visited matrix
    if(next_orientation < 0):
        next_orientation+=360
    next_orientation%=360
    
    next_node.append(next_x)
    next_node.append(next_y)
    next_node.append(next_orientation)
    return step_size, tuple(next_node)

# Next moves
def move_pos_thirty(current_node, step_size):  
    next_node = []
    theta = 30
    next_x = int(round((current_node[0] + step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
    next_y = int(round((current_node[1] + step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
    next_orientation = theta + current_node[2]
    
    # For negative index in the visited matrix
    if(next_orientation < 0):
        next_orientation+=360
    next_orientation%=360
    
    next_node.append(next_x)
    next_node.append(next_y)
    next_node.append(next_orientation)
    return step_size, tuple(next_node)

# Next moves
def move_pos_sixty(current_node, step_size):  
    next_node = []
    theta = 60
    next_x = int(round((current_node[0] + step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
    next_y = int(round((current_node[1] + step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
    next_orientation = theta + current_node[2]
    
    # For negative index in the visited matrix
    if(next_orientation < 0):
        next_orientation+=360
    next_orientation%=360
    
    next_node.append(next_x)
    next_node.append(next_y)
    next_node.append(next_orientation)
    return step_size, tuple(next_node)

# Function to check the validity of moves made
def valid_next_steps(current_node, obstacle_map, step_size):
    valid_neighbors = []
    neg_sixty = move_neg_sixty(current_node, step_size)
    neg_thirty = move_neg_thirty(current_node, step_size)
    zero = move_zero(current_node, step_size)
    pos_thirty = move_pos_thirty(current_node, step_size)
    pos_sixty = move_pos_sixty(current_node, step_size)
    
    if((neg_sixty[1][0] - 5)< 0 or (neg_sixty[1][0] + 5) > 599 
       or (neg_sixty[1][1] - 5) < 0 or (neg_sixty[1][1] + 5) > 249
       or obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0] + 5][0]==255
       or obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0] + 5][0]==255
       or obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0] - 5][0]==255
       or obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0] - 5][0]==255
       or obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0]][0]==255
       or obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0]][0]==255
       or obstacle_map[neg_sixty[1][1]][neg_sixty[1][0] - 5][0]==255
       or obstacle_map[neg_sixty[1][1]][neg_sixty[1][0] - 5][0]==255
       or sum(obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0] + 5])==765
       or sum(obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0] + 5])==765
       or sum(obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0] - 5])==765
       or sum(obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0] - 5])==765
       or sum(obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0]])==765
       or sum(obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0]])==765
       or sum(obstacle_map[neg_sixty[1][1]][neg_sixty[1][0] - 5])==765
       or sum(obstacle_map[neg_sixty[1][1]][neg_sixty[1][0] - 5])==765
       ):
        print("-60 move invalid")
    else:
        valid_neighbors.append(neg_sixty)
        
    if((neg_thirty[1][0] - 5)< 0 or (neg_thirty[1][0] + 5) > 599 
       or (neg_thirty[1][1] - 5) < 0 or (neg_thirty[1][1] + 5) > 249
       or obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0] + 5][0]==255
       or obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0] - 5][0]==255
       or obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0] + 5][0]==255
       or obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0] - 5][0]==255
       or obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0]][0]==255
       or obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0]][0]==255
       or obstacle_map[neg_thirty[1][1]][neg_thirty[1][0] + 5][0]==255
       or obstacle_map[neg_thirty[1][1]][neg_thirty[1][0] - 5][0]==255
       or sum(obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0] + 5])==765
       or sum(obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0] - 5])==765
       or sum(obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0] + 5])==765
       or sum(obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0] - 5])==765
       or sum(obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0]])==765
       or sum(obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0]])==765
       or sum(obstacle_map[neg_thirty[1][1]][neg_thirty[1][0] + 5])==765
       or sum(obstacle_map[neg_thirty[1][1]][neg_thirty[1][0] - 5])==765
       ):
        print("-30 move invalid")
    else:
        valid_neighbors.append(neg_thirty)
        
    if((zero[1][0] - 5)< 0 or (zero[1][0] + 5) > 599 
       or (zero[1][1] - 5) < 0 or (zero[1][1] + 5) > 249
       or obstacle_map[zero[1][1] + 5][zero[1][0] + 5][0]==255
       or obstacle_map[zero[1][1] + 5][zero[1][0] - 5][0]==255
       or obstacle_map[zero[1][1] - 5][zero[1][0] + 5][0]==255
       or obstacle_map[zero[1][1] - 5][zero[1][0] - 5][0]==255
       or obstacle_map[zero[1][1]][zero[1][0] + 5][0]==255
       or obstacle_map[zero[1][1]][zero[1][0] - 5][0]==255
       or obstacle_map[zero[1][1] + 5][zero[1][0]][0]==255
       or obstacle_map[zero[1][1] - 5][zero[1][0]][0]==255     
       or sum(obstacle_map[zero[1][1] + 5][zero[1][0] + 5])==765
       or sum(obstacle_map[zero[1][1] + 5][zero[1][0] - 5])==765
       or sum(obstacle_map[zero[1][1] - 5][zero[1][0] + 5])==765
       or sum(obstacle_map[zero[1][1] - 5][zero[1][0] - 5])==765
       or sum(obstacle_map[zero[1][1]][zero[1][0] + 5])==765
       or sum(obstacle_map[zero[1][1]][zero[1][0] - 5])==765
       or sum(obstacle_map[zero[1][1] + 5][zero[1][0]])==765
       or sum(obstacle_map[zero[1][1] - 5][zero[1][0]])==765  
       ):
        print("0 move invalid")
    else:
        valid_neighbors.append(zero)
        
    if((pos_thirty[1][0] - 5)< 0 or (pos_thirty[1][0] + 5) > 599 
       or (pos_thirty[1][1] - 5) < 0 or (pos_thirty[1][1] + 5) > 249
       or obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0] + 5][0]==255
       or obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0] + 5][0]==255
       or obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0] - 5][0]==255
       or obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0] - 5][0]==255
       or obstacle_map[pos_thirty[1][1]][pos_thirty[1][0] + 5][0]==255
       or obstacle_map[pos_thirty[1][1]][pos_thirty[1][0] - 5][0]==255
       or obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0]][0]==255
       or obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0]][0]==255
       or sum(obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0] + 5])==765
       or sum(obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0] + 5])==765
       or sum(obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0] - 5])==765
       or sum(obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0] - 5])==765
       or sum(obstacle_map[pos_thirty[1][1]][pos_thirty[1][0] + 5])==765
       or sum(obstacle_map[pos_thirty[1][1]][pos_thirty[1][0] - 5])==765
       or sum(obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0]])==765
       or sum(obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0]])==765
       ):
        print("30 move invalid")
    else:
        valid_neighbors.append(pos_thirty)
        
    if((pos_sixty[1][0] - 5)< 0 or (pos_sixty[1][0] + 5) > 599 
       or (pos_sixty[1][1] - 5) < 0 or (pos_sixty[1][1] + 5) > 249
       or obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0] + 5][0]==255
       or obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0] - 5][0]==255
       or obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0] - 5][0]==255
       or obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0] + 5][0]==255
       or obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0]][0]==255
       or obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0]][0]==255
       or obstacle_map[pos_sixty[1][1]][pos_sixty[1][0] - 5][0]==255
       or obstacle_map[pos_sixty[1][1]][pos_sixty[1][0] + 5][0]==255  
       or sum(obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0] + 5])==765
       or sum(obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0] - 5])==765
       or sum(obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0] - 5])==765
       or sum(obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0] + 5])==765
       or sum(obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0]])==765
       or sum(obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0]])==765
       or sum(obstacle_map[pos_sixty[1][1]][pos_sixty[1][0] - 5])==765
       or sum(obstacle_map[pos_sixty[1][1]][pos_sixty[1][0] + 5])==765       
       ):
        print("60 move invalid")
    else:
        valid_neighbors.append(pos_sixty) 
    return valid_neighbors  

def astar_search(obstacle_map):
    while open_list:
        current_node = open_list.get()[1]
        print("current_node: ", current_node)
        close_list[int(round(current_node[1] * 2)),int(round(current_node[0] * 2)), int(round(current_node[2]/30))] = 2
        visual_list.append(current_node)
        goal_reached = False
        if(math.sqrt((current_node[0] - goal[0])**2 + (current_node[1] - goal[1])**2) < 0.5 and 
           current_node[2] == goal[2]):
            goal_reached = True
            print(goal_reached)
        else:
            goal_reached = False
            print(goal_reached)
            
        if (goal_reached):
            print("Reached the given goal")
            back_track_path(current_node)
            break
        else:
            valid_neighbors = valid_next_steps(current_node, obstacle_map, step_size)
            for next_node in valid_neighbors:
                # print("Next Node: ", next_node)
                closed = close_list[int(round(next_node[1][1] * 2)),int(round(next_node[1][0] * 2)), int(round(next_node[1][2]/30))]
                # if the node is already present, ignore it
                if(closed == 2):
                    print("visited: ", next_node)
                    continue    
                else:
                    cost_to_next_node = next_node[0]
                    total_cost_to_node = cost_of_node[current_node] + cost_to_next_node
                    if(next_node[1] not in cost_of_node or total_cost_to_node < cost_of_node[next_node[1]]):
                        cost_of_node[next_node[1]] = total_cost_to_node
                        # Adding Euclidean heuristic
                        euc_dist = math.sqrt((next_node[1][0] - goal[0])**2 + (next_node[1][1] - goal[1])**2)
                        cost_with_heuristic = total_cost_to_node + euc_dist       
                        open_list.put((cost_with_heuristic, next_node[1]))
                        parent_node[next_node[1]] = current_node   
                        
# Function to back track and obtain the shortest path possible
def back_track_path(current_node):
    final_node = goal
    while final_node != None:
        found_path.append(final_node)
        final_node = parent_node[final_node]
    # the path found will be appended in reverse
    found_path.reverse()