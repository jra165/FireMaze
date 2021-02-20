#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 19 22:00:43 2021

@author: joshua.atienza
"""

import heapq as pq 

#maze = genMaze(5,0.1)
#print(maze)

def aStar(start,goal):
  dist =  (np.abs(goal[1]-start[1])**2 + (np.abs(goal[0]-start[0])**2))**1/2
  fringe = [(dist, 0, dist, start, (-1,-1))]  # cost, node, parent
  pq.heapify(fringe)  


  closed = set([])

  movements = [(1,0),(-1,0),(0,1),(0,-1)]

  map = [[(0,0) for i in range(len(maze))] for i in range(len(maze))]
  parentFringe = [(-1,-1)]



  while (len(fringe)>0):

    f, g, h, current, parent = pq.heappop(fringe)

    if current in closed:
      continue

    map[current[0]][current[1]] = parent

    if (current[0] ==goal[0] and current[1] == goal[1]):
      break

  

    validNeigh = []
    validPar = []

 
    for i in range(len(movements)):
      distX, distY = movements[i]
      x = current[0] + distX
      y = current[1] + distY 

      if (x<0 or y < 0 or x >= len(maze) or y >= len(maze) or maze[x][y] == 1 or (x,y) in closed):
        #print(x,y)
        continue

          #if (not (set(current).issubset(closed))):
      #print(x,y)
      validNeigh.append((x,y))
      validPar.append(current)

      dist =  (np.abs(goal[1]-y)**2 + (np.abs(goal[0]-x)**2))**1/2
      pq.heappush(fringe, (g+1+dist, g+1, dist, (x,y), current))
  
    
    closed.add(current)

  path = []
  node = goal
  if (map[node[0]][node[1]]==(0,0)):
    return 0, len(closed)#no shortest path found

  while True: 
    path.append(node)
    node = map[node[0]][node[1]]
    #print(node)
    if node[0] == -1:
      break
  
  path = path[::-1]
  return path, len(closed)


maze = genMaze(250,0.2)
path = aStar((0,0),(len(maze)-1,len(maze)-1))
if (path[0] is not 0):
  print(path[0])
  print(markPath(maze,path[0]))
else:
  print("No path found")
#shortestBFS((0,0), (len(maze)-1, len(maze)-1))

