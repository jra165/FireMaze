#Name: Joshua Atienza, Janet Zhang
#NetID: jra165, jrz42
#Project 1: Maze on Fire

#Question 1 - Generate maze
#import sys
import numpy as np

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


#Testing
print(generate_maze(3, 0.1))

