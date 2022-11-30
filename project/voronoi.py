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
[x] Identify boundary vertices and add edges b/w them
[x] Find a reachable corner point from src and dest
[x] constuct a path from src corner point to dest corner point using bfs
"""

import unittest
import random
from copy import deepcopy
import numpy as np
import generate
from graph import Graph
from collections import deque

def distance(dx, dy):
    return np.sqrt(dx**2 + dy**2)

def num_digits(n):
    """
    returns number of digits in n
    """
    if n==0:
        return 1
    count=0
    while n:
        count += 1
        n //= 10
    return count

def interpolate(a, b):
    """
    returns the list of points on the line segment joining a and b
    """
    x1, y1 = a
    x2, y2 = b

    if x1==x2:
        # vertical line
        points = [(x1, t) for t in range(y1, y2, 1 if y2>y1 else -1)]
    elif y1==y2:
        # horizontal line
        points = [(t, y1) for t in range(x1, x2, 1 if x2>x1 else -1)]
    else:
        # slanting line
        slope = np.arctan((y2-y1)/(x2-x1)) # unit: radians
        if x2<x1:
            if y2>y1:
                slope = np.pi + slope
            else:
                slope = slope - np.pi
        # print(f"slope in degrees: {slope * 180 / np.pi}")
        dist = int(np.ceil(distance(x2-x1, y2-y1)))
        # print(f"distance: {dist}")
        points = set()
        for d in range(dist):
            x = int(round(x1 + d * np.cos(slope)))
            y = int(round(y1 + d * np.sin(slope)))
            points.add((x, y))
        points = list(points)
        points.sort()
    # print(f"points b/w {a} and {b}: {points}")
    return points

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

        self.graph = Graph()
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
        self.neighbouringRegions = [] # list of sets of obstacleIds for which a vertex is a corner
        for i in range(self.m + 1):
            for j in range(self.n + 1):
                isCornerVertex, regions = self.get_neighbours(i, j)
                if isCornerVertex:
                    self.graph.add_vertex((i, j))
                    self.neighbouringRegions.append(regions)

        # add edge b/w vertices with >=2 common neighbours
        for i in range(self.graph.numVertices - 1):
            for j in range(i+1, self.graph.numVertices):
                intersection = self.neighbouringRegions[i].intersection(self.neighbouringRegions[j])
                if len(intersection)>=2:
                    self.graph.add_edge(i, j)
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
        print(f"i: (x, y) | self.graph.edges[i]")
        for i, (x, y) in enumerate(self.graph.vertex):
            print(f"{i}: ({x}, {y}) | {self.graph.edges[i]}")

    def is_path_clear(self, src, dest):
        """
        returns True iff there is no obstacle on the straight line path from src to dest
        note: return True conservatively, in case of doubt return False since there are multiple candidates
        assumption: src/dest coordinates are valid
        """
        points = interpolate(src, dest)
        # print(f"points: {points}")
        distanceThreshold = 1 # minimum allowed distance from obstacle
        for x,y in points:
            if distance(self.margin[x][y][1], self.margin[x][y][2]) < distanceThreshold:
                return False
        return True

    def cornerVertex(self, coordinate):
        """
        returns one of the corners of the obstacle boundary in which the coordinate is present. The corner is such that it can be reached from the coordinate w/o hitting any obstacle.
        """
        x, y = coordinate

        if self.map[x][y]>0:
            raise Exception("obstacle at src location")
        obstacleId = self.margin[x][y][0]
        candidates = []
        for i in range(self.graph.numVertices):
            if obstacleId in self.neighbouringRegions[i]:
                candidates.append(i)

        for c in candidates:
            vertex = self.graph.vertex[c]
            # ensure that vertex coordinate lies within pixel grid
            if vertex[0]==self.m:
                vertex = (vertex[0]-1, vertex[1])
            if vertex[1]==self.n:
                vertex = (vertex[0], vertex[1]-1)
            if self.is_path_clear((x, y), vertex):
                return c

        raise Exception("no valid path from src to dest")

    def path(self, src, dest):
        """
        returns the list of waypoints to approach in order to reach the destination from src
        """
        if not (0<=src[0]<self.m and 0<=src[1]<self.n):
            raise Exception("invalid src coordinate")
        if not (0<=dest[0]<self.m and 0<=dest[1]<self.n):
            raise Exception("invalid dest coordinate")

        # srcV is a vertex that can be reached from src w/o hitting any obstacle
        srcV = self.cornerVertex(src)

        # destV is a vertex that can be reached from dest w/o hitting any obstacle
        destV = self.cornerVertex(dest)

        graphPath = self.graph.path(srcV, destV)
        transformedPath = self.graph.transform(graphPath)
        return [src] + transformedPath + [dest]

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

    def test_2(self):
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

    def test_4a(self):
        grid = np.array([
            [0,0,0,0,0,0,1,0,1,0],
            [0,0,1,0,0,0,1,0,0,0],
            [0,0,1,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,1],
            [0,0,0,0,0,0,1,0,0,1],
            [0,0,0,0,1,1,0,0,0,0]
        ])
        voronoi = Voronoi(grid)
        # voronoi.print_grid()
        # voronoi.print_map()
        voronoi.print_margin()
        voronoi.print_boundary()
        src, dest = (3, 1), (6, 9)
        ans = [src] + [(5, 5), (5, 7), (4, 9)] + [dest]
        path = voronoi.path(src, dest)
        print(f"path: {path}")
        self.assertEqual(path, ans)

    def test_4b(self):
        grid = np.array([
            [0,0,0,0,0,0,1,0,1,0],
            [0,0,1,0,0,0,1,0,0,0],
            [0,0,1,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,1],
            [0,0,0,0,0,0,1,0,0,1],
            [0,0,0,0,1,1,0,0,0,0]
        ])
        voronoi = Voronoi(grid)
        # voronoi.print_grid()
        # voronoi.print_map()
        voronoi.print_margin()
        voronoi.print_boundary()
        src, dest = (2, 9), (8, 2)
        ans = [src] + [(4, 9), (5, 7), (5, 5)] + [dest]
        path = voronoi.path(src, dest)
        print(f"path: {path}")
        self.assertEqual(path, ans)

    def test_4c(self):
        grid = np.array([
            [0,0,0,0,0,0,1,0,1,0],
            [0,0,1,0,0,0,1,0,0,0],
            [0,0,1,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,1],
            [0,0,0,0,0,0,1,0,0,1],
            [0,0,0,0,1,1,0,0,0,0]
        ])
        voronoi = Voronoi(grid)
        # voronoi.print_grid()
        # voronoi.print_map()
        voronoi.print_margin()
        voronoi.print_boundary()
        src, dest = (9, 3), (9, 9)
        ans = [src] + [(5, 5), (5, 7), (10, 8)] + [dest]
        path = voronoi.path(src, dest)
        print(f"path: {path}")
        self.assertEqual(path, ans)

    def test_random(self):
        m,n = 10, 10
        numOnes = 10
        grid = generate.generate_random_grid(m, n, numOnes)
        voronoi = Voronoi(grid)
        voronoi.print_grid()
        # voronoi.print_map()
        voronoi.print_margin()
        voronoi.print_boundary()

if __name__ == '__main__':
    unittest.main()