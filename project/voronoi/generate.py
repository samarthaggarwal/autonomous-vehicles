import random
import numpy as np
from copy import deepcopy

def generate_random_grid(m, n, numOnes=3):
    """
    returns an mxn grid with obstacles located at random locations
    eg.
    [
        [0,1,0,0],
        [0,0,0,0],
        [0,0,1,0],
        [1,0,0,0]
    ]
    """
    # return np.random.randint(2, size=(m, n))
    if numOnes > m*n:
        raise Exception("numOnes > m * n, not possible")

    grid = np.zeros((m,n), dtype=int)
    while numOnes:
        # choose random index
        x = random.randrange(m)
        y = random.randrange(n)
        
        # assign randomly chosen index as obstacle (1) if it is an empty space
        if grid[x][y]==1:
            continue
        grid[x][y] = 1
        numOnes-=1
    return grid

def convert_to_obstacle_map(grid):
    """
    (returns connected components)
    The input grid of 0/1s can have a single obstacle being represented by multiple neighbouring 1s. This function join neighbouring 1s to form a single obstacle. Return the obstacle map.
    eg.
    [
        [0,1,0,0],
        [0,0,0,0],
        [0,0,2,0],
        [3,0,0,0]
    ]
    """
    map = deepcopy(grid)
    m, n = len(map), len(map[0])
    q = []
    obstacleId = 2 # starts from 2 since 0,1 already denote empty space and obstacle
    for i in range(m):
        for j in range(n):
            if map[i][j]!=1:
                continue
            q.append((i, j))
            # mark entire connected component with same obstacleId
            while q:
                x, y = q.pop()
                map[x][y] = obstacleId
                ### rectangular neighbours
                if x>0 and map[x-1][y]==1:
                    q.append((x-1, y))
                if x<m-1 and map[x+1][y]==1:
                    q.append((x+1, y))
                if y>0 and map[x][y-1]==1:
                    q.append((x, y-1))
                if y<n-1 and map[x][y+1]==1:
                    q.append((x, y+1))
                ### diagonal neighbours
                if x>0 and y>0 and map[x-1][y-1]==1:
                    q.append((x-1, y-1))
                if x>0 and y<n-1 and map[x-1][y+1]==1:
                    q.append((x-1, y+1))
                if x<m-1 and y>0 and map[x+1][y-1]==1:
                    q.append((x+1, y-1))
                if x<m-1 and y<n-1 and map[x+1][y+1]==1:
                    q.append((x+1, y+1))
            # update obstacleId
            obstacleId+=1
    # reduce obstacleId so that obstacleId starts from 1
    for i in range(m):
        for j in range(n):
            if map[i][j]>0:
                map[i][j] -= 1
    numObstacles = obstacleId - 2
    return map, numObstacles

def generate_random_map(m, n, numOnes=5):
    grid = generate_random_grid(m, n, numOnes)
    map, numObstacles = convert_to_obstacle_map(grid)
    return map, numObstacles
