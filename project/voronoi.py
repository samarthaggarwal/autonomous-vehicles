"""
This module plans the path from source to destination with obstacle avoidance.

Inputs
    - 2D grid of 0/1 denoting open space (0) and obstacles (1)
    - source (current location)
    - destination

Outputs
    - next coordinate to approach

Steps
[x] Convert pixel grid to obstacle map (join connected components)
[x] Make voronoi diagram
[x] Identify boundary vertices
[ ] Find closest points from src/dest to voronoi path (drop perpendicars to all edges and pick the shortest one)
[ ] find next point (closest perpendicular point if src is not already there, o/w djikstra/bfs from src to destination)
"""

import unittest
import random
from copy import deepcopy
import numpy as np
import generate
from collections import deque

def distance(dx, dy):
    return np.sqrt(dx**2 + dy**2)

def num_digits(n):
    if n==0:
        return 1
    count=0
    while n:
        count += 1
        n //= 10
    return count

class Voronoi:
    def __init__(self, grid):
        """
        grid: 2D pixel grid
        construct a voronoi diagram
        eg.
        [
            [-1, 1,-1,-1],
            [-1,-1,-2,-2],
            [-3,-2, 2,-2],
            [ 3,-3,-2,-2]
        ]
        """
        self.grid = grid
        self.m, self.n = len(grid), len(grid[0])

        self.map, self.numObstacles = generate.convert_to_obstacle_map(grid)
        
        # self.margin contains closest obstacle and distance from it for every point
        self.margin = []
        for i in range(self.m):
            self.margin.append([(0, np.inf, np.inf)] * self.n) # (closestObstacleId, displacementX, displacementY)
        self.dilate_obstacles()

        self.vertices = []
        self.edges = []
        self.generate_boundaries()

        return

    def is_obstacle_boundary(self, x, y):
        """
        returns True iff (x,y) is the boundary of an obstacle i.e. (x,y) has no neighbour that is not an obstacle.
        """
        if not (0<=x<=self.m and 0<=y<=self.n):
            raise Exception("Invalid point, outside map boundaries")
        if self.map[x][y]==0:
            # not an obstacle
            return False
        surrounded = (x==0 or self.map[x-1][y]>0) and \
                    (x==self.m-1 or self.map[x+1][y]>0) and \
                    (y==0 or self.map[x][y-1]>0) and \
                    (y==self.n-1 or self.map[x][y+1]>0)
        return not surrounded

    def dilate_obstacles(self):
        """
        populates self.margin
        """
        if self.numObstacles==0:
            return

        q = deque()

        # update margin for obstacles
        for i in range(self.m):
            for j in range(self.n):
                if self.map[i][j]>0:
                    self.margin[i][j] = (self.map[i][j], 0, 0)
                    if self.is_obstacle_boundary(i, j):
                        q.append((i, j))

        while q:
            x, y = q.popleft()
            obstacleId, dx, dy = self.margin[x][y]
            if x>0 and distance(self.margin[x-1][y][1], self.margin[x-1][y][2]) > distance(dx-1, dy):
                self.margin[x-1][y] = (obstacleId, dx-1, dy)
                q.append((x-1, y))
            if x<self.m-1 and distance(self.margin[x+1][y][1], self.margin[x+1][y][2]) > distance(dx+1, dy):
                self.margin[x+1][y] = (obstacleId, dx+1, dy)
                q.append((x+1, y))
            if y>0 and distance(self.margin[x][y-1][1], self.margin[x][y-1][2]) > distance(dx, dy-1):
                self.margin[x][y-1] = (obstacleId, dx, dy-1)
                q.append((x, y-1))
            if y<self.n-1 and distance(self.margin[x][y+1][1], self.margin[x][y+1][2]) > distance(dx, dy+1):
                self.margin[x][y+1] = (obstacleId, dx, dy+1)
                q.append((x, y+1))

        return

    def get_neighbours(self, x, y):
        """
        returns neighbours of vertex at (x, y).
        note: m*n grid has (m+1)*(n+1) vertices
        """
        touchesBoundary = x==0 or x==self.m or y==0 or y==self.n
        neighbours = set()
        if x>0 and y>0:
            neighbours.add(self.margin[x-1][y-1][0])
        if x<self.m and y>0:
            neighbours.add(self.margin[x][y-1][0])
        if x>0 and y<self.n:
            neighbours.add(self.margin[x-1][y][0])
        if x<self.m and y<self.n:
            neighbours.add(self.margin[x][y][0])

        isCornerVertex = len(neighbours) >= 2 if touchesBoundary else len(neighbours) >= 3
        return isCornerVertex, neighbours

    def generate_boundaries(self):
        """
        identify corner vertices and generate a graph of edges connecting them.
        """
        neighbouringRegions = [] # list of sets
        for i in range(self.m + 1):
            for j in range(self.n + 1):
                isCornerVertex, regions = self.get_neighbours(i, j)
                if isCornerVertex:
                    self.vertices.append((i, j))
                    self.edges.append([])
                    neighbouringRegions.append(regions)

        # add edge b/w vertices with >=2 common neighbours
        numVertices = len(self.vertices)
        for i in range(numVertices-1):
            for j in range(i+1, numVertices):
                intersection = neighbouringRegions[i].intersection(neighbouringRegions[j])
                if len(intersection)>=2:
                    self.edges[i].append(j)
                    self.edges[j].append(i)
        return

    def print_grid(self):
        print("===== grid =====")
        for i in range(self.m):
            for j in range(self.n):
                print(self.grid[i][j], end="")
            print()

    def print_map(self):
        print("===== map =====")
        spaces = 1 + num_digits(self.numObstacles)
        for i in range(self.m):
            for j in range(self.n):
                print(" " * (spaces - num_digits(self.map[i][j])) + str(self.map[i][j]), end="")
            print()

    def print_margin(self):
        print("===== margin =====")
        spaces = 1 + num_digits(self.numObstacles)
        for i in range(self.m):
            for j in range(self.n):
                isObstacle = self.margin[i][j][1]==0 and self.margin[i][j][2]==0
                if isObstacle:
                    text = "\033[4m" + str(self.margin[i][j][0]) + "\033[0m" # to underline obstacle points
                else:
                    text = str(self.margin[i][j][0])
                print(" " * (spaces - num_digits(self.margin[i][j][0])) + text, end="")
            print()

    def print_boundary(self):
        print("===== boundary =====")
        print(f"i: (x, y) | self.edges[i]")
        for i, (x, y) in enumerate(self.vertices):
            print(f"{i}: ({x}, {y}) | {self.edges[i]}")

    def nextTarget(self, src, dest):
        """
        returns the next points to approach
        """
        raise Exception("unimplemented")

class TestVoronoi(unittest.TestCase):
    def test_1(self):
        grid = np.array([
            [0,1,0,0],
            [0,0,0,0],
            [0,0,1,0],
            [1,0,0,0]
        ])
        voronoi = Voronoi(grid)
        voronoi.print_grid()
        voronoi.print_margin()
        # src = (0, 0)
        # dest = (m-1, n-1)
        # nextTarget = voronoi.nextTarget(src, dest)
        # answer = (0, 1)
        # self.assertEqual(nextTarget, answer)

    def test_2(self):
        """
        test a specific input grid
        """
        grid = np.array([
            [0,0,1,0,0,0,0,0,1,1],
            [0,0,0,1,0,0,1,1,0,0],
            [0,0,0,0,0,0,0,1,1,0],
            [0,0,0,0,0,0,1,0,0,0],
            [1,0,1,1,0,0,0,1,1,0],
            [0,1,0,0,0,0,0,0,0,0],
            [0,1,0,0,0,0,1,0,0,1],
            [0,1,0,0,0,0,1,1,1,1],
            [0,1,0,0,0,0,1,1,0,0],
            [1,0,0,0,0,0,1,0,1,1]
        ])
        voronoi = Voronoi(grid)
        voronoi.print_grid()
        # voronoi.print_map()
        voronoi.print_margin()
        voronoi.print_boundary()

    def test_3(self):
        grid = np.array([
            [0,1,0,0],
            [0,0,0,0],
            [0,0,0,1],
            [1,0,0,0]
        ])
        voronoi = Voronoi(grid)
        voronoi.print_grid()
        voronoi.print_margin()
        voronoi.print_boundary()

    def test_random(self):
        m,n = 20, 20
        numOnes = 10
        grid = generate.generate_random_grid(m, n, numOnes)
        voronoi = Voronoi(grid)
        voronoi.print_grid()
        # voronoi.print_map()
        voronoi.print_margin()
        voronoi.print_boundary()

if __name__ == '__main__':
    unittest.main()