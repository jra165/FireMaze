#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 11:54:32 2021

@author: Joshua Atienza, Janet Zhang
@net-id: jra165, jrz42
@project: Maze on Fire
"""

#Imports
#import sys
import numpy as np
from matplotlib import pyplot as plt
import heapq as pq
import math
from copy import deepcopy

"""
Problem 1: Write an algorithm for generating a maze with a given dimension 
and obstacle density p.
"""
#Function for generating our maze
def generate_maze(dim, p):
    
    #Initialize maze with all 0s
    maze = [[0] * dim for i in range(dim)]
    
    #Loop through each cell
    for y in range(len(maze)):
        
        for x in range(len(maze[0])):
        
            #Disregard start and target cell
            if((y == 0 and x == 0) or (y == dim - 1 and x == dim - 1)):
                continue

            #If random number falls within threshold, set cell as obstacle
            if(np.random.random() <= p):
                maze[x][y] = 1

    return maze



"""
Problem 2: Implement DFS between any 2 points
"""
#Function for running DFS on maze
def DFS(maze, start, target):
    
    #List of cell's valid neighbors
    neighbors = []
    
    #Possible movements in the maze
    directions = [[0,1], [0,-1], [1,0], [-1,0]]
    
    #Initialize our fringe and visited
    fringe = [start]
    visited = set([])
    
    #Loop until fringe is empty
    while(len(fringe) > 0):
        
        cur = fringe.pop()
        
        #Check if node has been visited to remove redundancy
        if(cur in visited): 
            continue
        
        #Path is possible if coordinates of current match target
        if (cur[0] ==target[0] and cur[1] == target[1]):
            return True
    
        
        #Check all four directions
        for i in range(len(directions)):
            
            #Use the direction array
            a = cur[0] + directions[i][0]
            b = cur[1] + directions[i][1]
            
            #Not blocked and valid
            if(a < 0 or b < 0 or a >= len(maze[0]) or b >= len(maze)
               or maze[a][b] == 1 or (a,b) in visited):
               continue
           
            #Add to neighbors
            neighbors.append((a,b))
        
        #Add to fringe
        fringe += neighbors
        
        #Add current node to visited set
        visited.add(cur)
        
    #No path possible
    return False
            
#Function for plotting probabilities against each other
def probability_plot():
    
    prob = 0
    obstacle_probs = []
    goal_probs = []
    
    path_count = 0
    
    #Create list of probabilities, incrementing by 0.1
    for i in range(10):
        obstacle_probs.append(prob)
        prob += 0.1
    
    #Loop through each obstacle density
    for i in range(len(obstacle_probs)):
        
        #Repeat 40 times for each probability
        for j in range(40):
            
            maze = generate_maze(100, obstacle_probs[i])
            if(DFS(maze, (0,0), (len(maze)-1, len(maze)-1))):
                path_count += 1
        
        #Get averages for each resulting goal probability
        goal_probs.append(path_count / 40)
        path_count = 0
    
    #Plot lists against each other
    plt.plot(obstacle_probs, goal_probs)
    plt.xlabel("Obstacle density p")
    plt.ylabel("probability that S can be reached from G")
    
    #Create a dictionary following format
    #key = object density
    #value = probability you can go from S to G successfully
    res = {} 
    for key in obstacle_probs: 
        for value in goal_probs: 
            res[key] = value 
            goal_probs.remove(value) 
            break 
        
    return res

"""
Testing for DFS and probability plot
"""

#Testing
#Generate maze
m = generate_maze(4, 0.1)
print(m)

#Run DFS
print(DFS(m, (0,0), (3,3)))

#Run probability plot
print(probability_plot())

"""
We would prefer DFS over BFS in this case because the objective is to determine if a 
path exists, and not to look for the shortest path.
In our case, we just want to keep going down possible path until an obstacle is hit 
or our target is reached.
"""


"""Problem 3: Implement BFS, A* to determine shortest path
"""
#Function for running BFS on maze
def BFS(maze, start, target):
    
    #Possible movements in the maze
    directions = [[0,1], [0,-1], [1,0], [-1,0]]
    
    #Initialize queue
    fringe = [start]
    visited = set([])
    
    parent_map = [[(0,0) for i in range(len(maze))] for i in range(len(maze))]
    parent_fringe = [(-1,-1)]
    
    #Search until the goal is reached
    while(len(fringe) > 0):
        
        cur = fringe[0]
        fringe.pop(0)
        
        #Pop from top of parent stack
        cur_parent = parent_fringe[0]
        parent_fringe.pop(0)
        
        #Check if node has been visited to remove redundancy
        if(cur in visited): 
            continue
        
        #Mark node as visited, update in parent_map
        parent_map[cur[0]][cur[1]] = cur_parent
        
        #Check if node is the goal
        if (cur[0] == target[0] and cur[1] == target[1]):
            print("Goal found! Breaking.")
            break
        
        #Expand cur and validate neighbors/parents, will use when checking all 4 directions
        neighbors = []
        parents = []
        
        #Check all 4 directions
        for i in range(len(directions)):
            
            a = cur[0] + directions[i][0]
            b = cur[1] + directions[i][1]
            
            #Not blocked and valid
            if(a < 0 or b < 0 or a >= len(maze[0]) or b >= len(maze)
               or maze[a][b] == 1 or (a,b) in visited):
               continue
           
            #Add to valid neighbors
            neighbors.append((a,b))
            parents.append(cur)
        
        #Update fringes
        fringe += neighbors
        parent_fringe += parents
        visited.add(cur)
       
    #Plot the path
    path = []
    node = target

    #No possible path
    if (parent_map[node[0]][node[1]]==(0,0)):
        print("# of visited nodes: " + str(len(visited)))
        return 0

    #Create path
    while True: 
        path.append(node)
        node = parent_map[node[0]][node[1]]
        if node[0] == -1:
            break
  
    #Return shortest path and number of visited nodes
    path = path[::-1]
    print("# of visited nodes: " + str(len(visited)))
    return path

    
"""
We would prefer DFS over BFS in this case because the objective is to determine if a 
path exists, and not to look for the shortest path.
In our case, we just want to keep going down possible path until an obstacle is hit 
or our target is reached.
"""
#Helper function for getting Euclidean distance
def get_distance(start, target):
    
    euc_dist = math.sqrt((target[1]-start[1])**2+((target[0]-start[0])**2))
    return euc_dist

#Function for running A* search on the maze
def a_Star(maze, start, target):

    #Possible movements in the maze
    directions = [[0,1], [0,-1], [1,0], [-1,0]]

    #Initialize closed list
    closed_list = set([])

    #Initialize open list and convert to heap to access smallest element
    open_list = [(get_distance(start, target), 0, get_distance(start, target), start, (-1,-1))]
    pq.heapify(open_list)
 
    #Initialize parent queue
    parent_map = [[(0,0) for i in range(len(maze))] for i in range(len(maze))]

    #Search until target is reached
    while(len(open_list) > 0):

        #Initialize parameters
        f, g, h, cur, par = pq.heappop(open_list)
        parent_map[cur[0]][cur[1]] = par
        
        #Expand cur and validate neihbors
        valid_parents = []
        valid_neighbors = []

        #If the target is reached, stop the search
        if(cur[0]==target[0] and cur[1] == target[1]):
            #print("Target reached.")
            break
        
        #Check if node visited to avoid redundancy
        if cur in closed_list: 
            continue
        
        #Check all four directions
        for i in range(len(directions)):

            #Use directions array to change the current position
            a = cur[0] + directions[i][0]
            b = cur[1] + directions[i][1]

            #Check if valid
            if(a >= len(maze) or b >= len(maze) or a < 0 or b < 0):
                continue
            if(maze[a][b] == 1):
                continue
            
            #Check if node on fire for strategy 2 implementation
            if(maze[a][b] == -1):
                continue
            
            #Check if visited
            if((a,b) in closed_list): 
                continue
            

            #Otherwise it is a valid move. Add the position and parent to the valid lists
            valid_parents.append(cur)
            valid_neighbors.append((a,b))
            
        
            #Push the new information to the heapified open list 
            pq.heappush(open_list,(g+1+get_distance((a,b), target),g+1,get_distance((a,b), target),(a,b),cur))
            
        
        #Push the current information to the closed list
        closed_list.add(cur)

    #Define the found path 
    path = []
    node = target

    #Check if there was a path found at all, returns 0 to indicate no path
    if(parent_map[node[0]][node[1]] == (0,0)):
        #print("# of visited nodes: " + str(len(closed_list)))
        print("No path found.")
        return 0
    
    #If there was, populate the path list with the correct positions
    while True:

        #Start from the end of the path, tracing target up the parent map
        path.append(node)
        node = parent_map[node[0]][node[1]]
        
        #Once target reaches the beginning, the path is finished
        if node[0] == -1:
            break

    #Return the found path
    path = path[::-1]
    #print("# of visited nodes: " + str(len(closed_list)))
    return path


"""
If there is no path from S to G, then the average ‘number of nodes explored by BFS 
- number of nodes explored by A∗’ would be zero, since they would need to check the
same amount of nodes to come to that conclusion. The larger the ‘obstacle density p’
the more likely the avg number of nodes would be large as well.  

"""

#Function for plotting density against BFS-A* node count against each other
def plot_BFS_ASTAR():
    
    prob = 0
    total = 0
    obstacle_probs = []
    node_count = []

    
    #Create list of obstacle densities, incrementing by 0.1
    for i in range(10):
        obstacle_probs.append(prob)
        prob += 0.1
    
    #Loop through each obstacle density
    for i in range(len(obstacle_probs)):
        
        #Repeat 40 times for each probability
        for j in range(40):
            
            maze = generate_maze(100, obstacle_probs[i])
            
            #Subtract nodes: BFS - A*
            difference = BFS(maze, (0,0), (len(maze)-1,len(maze)-1))[1] - a_Star(maze, (0,0), (len(maze)-1, len(maze)-1))[1]
            total += difference
        
        node_count.append(total / 40)
        total = 0
    
    #Plot lists against each other
    plt.plot(obstacle_probs, node_count)
    plt.xlabel("Obstacle density p")
    plt.ylabel("Average # nodes of BFS-A*")
    
    #Create a dictionary following format
    #key = object density
    #value = probability you can go from S to G successfully
    res = {} 
    for key in obstacle_probs: 
        for value in node_count: 
            res[key] = value 
            node_count.remove(value) 
            break 
        
    return res


"""
Testing for BFS, A*, and plots
"""

#Testing BFS
maze = generate_maze(5, 0.3)
print(maze)
print(BFS(maze, (0,0), (4,4)))


#Testing A* 
m = [[0, 0, 0, 1, 0],
    [1, 0, 0, 1, 1],
    [0, 0, 0, 1, 0],
    [1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0]
    ]
print(a_Star(m,(0,0),(4,4)))


#Run plot
print(plot_BFS_ASTAR())


"""
Question 4: Dimensions for DFS, BFS, A*
"""


"""
Question 5: Strategy 1, 2, 3
"""
#Function for advancing the fire one step
def advance_fire_one_step(maze, q):
    
    #Possible movements in the maze
    directions = [[0,1], [0,-1], [1,0], [-1,0]]
    
    #Copy original maze
    maze_copy = deepcopy(maze)
    
    #Loop through coordinates in maze
    for x in range(len(maze)):
        
        for y in range(len(maze[x])):
            
            #Count number of neighbors of (x,y) in current maze that are on fire 
            fire_count = 0
            
            #Disregard anything that's not a valid space
            if(maze[x][y] != 0):
                continue
            
            #Move all 4 directions
            for i in range(len(directions)):
                
                a = x + directions[i][0]
                b = y + directions[i][1]
                
                #Check if valid
                if(a < 0 or b < 0 or a >= len(maze) or b >= len(maze)):
                    continue
                
                #Increment fire counter
                if(maze[a][b] == -1):
                    fire_count += 1
            
            #Calculate probability of of step being on fire next step
            prob = 1 - (1-q)**fire_count
            if(np.random.random() <= prob):
                maze_copy[x][y] = -1
    
    return maze_copy



#Function for setting an initial, randomized step on fire
def set_on_fire(maze):
    
    #Generate a random coordinate as long as chosen step is valid
    while True:
        x = np.random.randint(len(maze))
        y = np.random.randint(len(maze))
    
        if maze[x][y] == 0:
            
            maze[x][y] = -1
            break
    
    return maze


"""
Set maze on fire testing
"""
#Advance fire one step test case
m = [[0, 0, 0, 1, 0],
    [1, 0, 0, 1, 1],
    [0, 0, 0, 1, 0],
    [1, 0, 0, 0, 0],
    [-1, 0, 1, 0, 0]
    ]
print(advance_fire_one_step(m, 0.8))

maze = generate_maze(4, 0.4)
#Print initial maze
print(maze)
#Print maze with 1 step on fire
print(set_on_fire(maze))


#Function for Strategy 1
def strategy_1(maze, q):
    
    #Initialize start and end points
    start = (0,0)
    target = (len(maze)-1, len(maze)-1)
    
    shortest_path = a_Star(maze, start, target)
    
    if shortest_path == 0:
        print("No path exists.")
        print("Final maze: ")
        print(np.matrix(maze))
        return 0
    
    set_on_fire(maze)
    print(np.matrix(maze))

    step_counter = 0
    for i in range(len(shortest_path)):
        
        
        cur = shortest_path[i]
        
        step_counter += 1
        
        if(maze[cur[0]][cur[1]] == -1):
            print("Agent burned!")
            print("Agent visited " + str(step_counter) + " nodes before catching on fire.")
            print("Final maze: ")
            print(np.matrix(maze))
            return -1
        
        maze = advance_fire_one_step(maze, q)
        print(np.matrix(maze))
    
    print("Agent survived!")
    print("Agent visited " + str(step_counter) + " nodes.")
    print("Final maze: ")
    print(np.matrix(maze))
    print(shortest_path)
    return 1

"""
Strategy 1 Testing
"""
mz = [[0, 0, 0, 1, 0],
    [1, 0, 0, 1, 1],
    [0, 0, 0, 1, 0],
    [1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0]
    ]
a = [[0, 0, 0, 1, 0],
    [1, 0, 0, 1, 1],
    [0, 0, 0, 1, 0],
    [1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0]
    ]
print(np.matrix(mz))
print(a_Star(a, (0,0), (4,4)))
print(strategy_1(mz, 0.4))
    
    
    
    
def strategy_2(maze, q):
    
    start = (0,0)
    target = (len(maze)-1, len(maze)-1)
    cur = start
    
    
    #Set initial fire to maze
    set_on_fire(maze)
    print("Initial maze: ")
    print(np.matrix(maze))
    
    #If the start is set on fire, them agent immediately burns
    if(maze[cur[0]][cur[1]] == -1):
        print("Agent burned!")
        print("Final maze: ")
        print(np.matrix(maze))
        return -1
    
    #Find initial shortest path from start
    shortest_path = a_Star(maze, start, target)
    
    #No path is available, return 0
    if shortest_path == 0:
        print("No path exists.")
        print("Final maze: ")
        print(np.matrix(maze))
        return 0

    #Take next step along shortest path until target is reached, not possible, or burns
    while cur != target:
        
        print("Current shortest path is: ")
        print(shortest_path)
        
        if(shortest_path == 0):
            print("No path exists!")
            print("Final maze: ")
            print(np.matrix(maze))
            return 0
        
        cur = shortest_path[1]
        maze = advance_fire_one_step(maze, q)
        print(np.matrix(maze))
        
        if(maze[cur[0]][cur[1]] == -1):
            print("Agent burned!")
            print("Final maze: ")
            print(np.matrix(maze))
            return -1
        
            
        shortest_path = a_Star(maze, cur, target)
       
    print("Agent survived!")
    print("Final maze: ")
    print(np.matrix(maze))
    return 1
            
    
mx = [[0, 0, 0, 1, 0],
      [1, 0, 0, 1, 1],
      [0, 0, 0, 1, 0],
      [1, 0, 0, 0, 0],
      [0, 0, 1, 0, 0]]  

a = generate_maze(20, 0)

print(strategy_2(a, 0.8))

    
    
    

    
    

        
            
            





