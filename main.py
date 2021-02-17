#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 11:54:32 2021

@author: Joshua Atienza, Janet Zhang
@net-id: jra165, jrz42
@project: Maze on Fire
"""


"""
Problem 1: Write an algorithm for generating a maze with a given dimension 
and obstacle density p.
"""
#import sys
import numpy as np
from matplotlib import pyplot as plt

def generate_maze(dim, p):
    
    #Initialize grid
    maze = np.zeros((dim, dim))
    
    #Loop through each cell
    for y in range(len(maze)):
        
        for x in range(len(maze[0])):
        
            #Disregard start and target cell
            if((y == 0 and x == 0) or (y == dim - 1 and x == dim - 1)):
                continue
            
            #Generate random number between 1 and 10
            rand = np.random.randint(1,10)

            #If random number falls within threshold, set cell as obstacle
            if(rand <= p * 10):
                maze[x][y] = 1

    return maze



"""
Problem 2: Implement DFS between any 2 points
"""
def DFS(maze, start, target):
    
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
        
        neighbors = []
        
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
            
#Plot probabilities against each other
def probability_plot():
    
    prob = 0
    obstacle_probs = []
    goal_probs = []
    
    path_count = 0
    
    for i in range(10):
        obstacle_probs.append(prob)
        prob += 0.1
    
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

#Testing
#Generate maze
m = generate_maze(4, 0.4)
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

def BFS(maze, start, goal):
    return False


def A_Star(maze, start, goal):
    return False






