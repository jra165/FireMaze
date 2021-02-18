#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 18 16:58:00 2021

@author: Joshua Atienza, Janet Zhang
@net-id: jra165, jrz42
@project: Maze on Fire
"""

import numpy as np
from matplotlib import pyplot as plt
import heapq as pq
import math
from copy import deepcopy
from main import a_Star, generate_maze, get_distance

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
def strategy_1(maze):
    
    #Initialize start and end points
    start = (0,0)
    target = (len(maze)-1, len(maze)-1)
    q = np.random.random()
    
    shortest_path = a_Star(maze, start, target)
    
    if shortest_path == 0:
        print("No path exists.")
        return 0, maze
    
    maze = set_on_fire(maze)
    
    for i in range(len(shortest_path)):
        
        cur = shortest_path[i]
        maze[cur[0]][cur[1]] = 2
        if(maze[cur[0]][cur[1]] == -1):
            print("Agent burned!")
            return -1, maze
        
        maze = advance_fire_one_step(maze, q)
        print(maze)
    
    return 1, maze

"""
Strategy 1 Testing
"""
mz = generate_maze(5, 0.2)
print(mz)
print(set_on_fire(mz))
print(advance_fire_one_step(mz, 0.5))
print(advance_fire_one_step(mz, 0.5))
print(strategy_1(mz))