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
from .generate import *
# from generate import *
from .graph import Graph
# from graph import Graph
from collections import deque
import time
import cv2
from matplotlib import pyplot as plt
from .utils import *

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
        # startTime = time.time()
        self.grid = grid
        self.m, self.n = len(grid), len(grid[0])

        self.map, self.numObstacles = convert_to_obstacle_map(grid)
        
        # self.margin contains closest obstacle and distance from it for every point
        self.margin = []
        for i in range(self.m):
            self.margin.append([(0, np.inf, np.inf)] * self.n) # (closestObstacleId, displacementX, displacementY)
        # beforeDilate = time.time()
        # print(f"before Dilate = {beforeDilate - startTime}")
        self.dilate_obstacles()
        # afterDilate = time.time()
        # print(f"after Dilate = {afterDilate - beforeDilate}")

        self.graph = Graph()
        self.generate_boundaries()
        # afterBoundary = time.time()
        # print(f"after Boundary = {afterBoundary - afterDilate}")

        return

    def plot_regions(self):
        """
        returns an m*n image such that region of every obstacle has a different color
        """
        # The 3 indicates that it is a color image with 3 channels (R, G, B)
        img = np.zeros((self.m, self.n, 3), dtype=np.uint8)

        # Generate random colors and store them in a list
        colors = []
        for i in range(self.numObstacles):
            r = random.randint(0, 255)
            g = random.randint(0, 255)
            b = random.randint(0, 255)
            colors.append((r, g, b))

        for i in range(self.m):
            for j in range(self.n):
                isObstacle = self.margin[i][j][1]==0 and self.margin[i][j][2]==0
                if isObstacle:
                    # mark obstacles with black
                    img[i][j] = (0, 0, 0)
                else:
                    # mark open spaces with color
                    img[i][j] = colors[self.margin[i][j][0] - 1]

        # cv2.imwrite("regions.png", img)
        return img

    def visualise_path(self, path):
        """
        plots the path on top of the obstacle regions
        """
        img = self.plot_regions()
        blue = (255, 0, 0)
        red = (0, 0, 255)
        yellow = (255, 255, 0)
        radius = 2
        thickness = 1
        for i in range(len(path)-1):
            img = cv2.line(img, path[i][::-1], path[i+1][::-1], blue, thickness)
            img = cv2.circle(img, path[i+1][::-1], radius, yellow, -1)

        img = cv2.circle(img, path[0][::-1], radius, blue, -1) # source
        img = cv2.circle(img, path[-1][::-1], radius, red, -1) # destination
        cv2.imwrite("path.png", img)
        # cv2.imshow("Voronoi Path", img)
        # cv2.waitKey(1)
        # plt.imshow(img)

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

    def is_obstacle_vertex(self, x, y):
        """
        returns True iff the vertex (x, y) lies on an obstacle.
        (x, y) is a point in the (m+1)*(n+1) grid of vertices.
        """
        return (x>0 and y>0 and self.map[x-1][y-1]>0) \
            or (x<self.m and y>0 and self.map[x][y-1]>0) \
            or (x>0 and y<self.n and self.map[x-1][y]>0) \
            or (x<self.m and y<self.n and self.map[x][y]>0)

    def generate_boundaries(self):
        """
        identify corner vertices and generate a graph of edges connecting them.
        also add 4 corners of image as vertices and edges along the image boundary.
        """
        self.neighbouringRegions = [] # list of sets of obstacleIds for which a vertex is a corner
        for i in range(self.m + 1):
            for j in range(self.n + 1):
                isCornerVertex, regions = self.get_neighbours(i, j)
                if isCornerVertex:
                    self.graph.add_vertex((i, j))
                    self.neighbouringRegions.append(regions)

        # add 4 corners of (m+1)*(n+1) grid
        corners = [(0, 0), (self.m, 0), (0, self.n), (self.m, self.n)]
        for corner in corners:
            if corner in self.graph.vertex:
                continue
            x, y = corner
            if self.is_obstacle_vertex(x, y):
                continue
            isCornerVertex, regions = self.get_neighbours(x, y)
            self.graph.add_vertex(corner)
            self.neighbouringRegions.append(regions)

        # add edge b/w vertices with >=2 common neighbours
        for i in range(self.graph.numVertices - 1):
            for j in range(i+1, self.graph.numVertices):
                intersection = self.neighbouringRegions[i].intersection(self.neighbouringRegions[j])
                if len(intersection)>=2 and self.is_path_clear(self.graph.vertex[i], self.graph.vertex[j]):
                    self.graph.add_edge(i, j)

        # add edges along grid boundaries
        top, bottom, left, right = [], [], [], []
        for i, v in enumerate(self.graph.vertex):
            if v[0]==0:
                top.append(i)
            if v[0]==self.m:
                bottom.append(i)
            if v[1]==0:
                left.append(i)
            if v[1]==self.n:
                right.append(i)
        top.sort(key=lambda x: self.graph.vertex[x])
        bottom.sort(key=lambda x: self.graph.vertex[x])
        left.sort(key=lambda x: self.graph.vertex[x])
        right.sort(key=lambda x: self.graph.vertex[x])
        for i in range(len(top)-1):
            a, b = self.graph.vertex[top[i]], self.graph.vertex[top[i+1]]
            a = (min(a[0], self.m-1), min(a[1], self.n-1))
            b = (min(b[0], self.m-1), min(b[1], self.n-1))
            if self.is_path_clear(a, b):
                self.graph.add_edge(top[i], top[i+1])
        for i in range(len(bottom)-1):
            a, b = self.graph.vertex[bottom[i]], self.graph.vertex[bottom[i+1]]
            a = (min(a[0], self.m-1), min(a[1], self.n-1))
            b = (min(b[0], self.m-1), min(b[1], self.n-1))
            if self.is_path_clear(a, b):
                self.graph.add_edge(bottom[i], bottom[i+1])
        for i in range(len(left)-1):
            a, b = self.graph.vertex[left[i]], self.graph.vertex[left[i+1]]
            a = (min(a[0], self.m-1), min(a[1], self.n-1))
            b = (min(b[0], self.m-1), min(b[1], self.n-1))
            if self.is_path_clear(a, b):
                self.graph.add_edge(left[i], left[i+1])
        for i in range(len(right)-1):
            a, b = self.graph.vertex[right[i]], self.graph.vertex[right[i+1]]
            a = (min(a[0], self.m-1), min(a[1], self.n-1))
            b = (min(b[0], self.m-1), min(b[1], self.n-1))
            if self.is_path_clear(a, b):
                self.graph.add_edge(right[i], right[i+1])

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
        distanceThreshold = 3 # minimum allowed distance from obstacle
        for x,y in points:
            x, y = min(x, self.m-1), min(y, self.n-1)
            if distance(self.margin[x][y][1], self.margin[x][y][2]) < distanceThreshold:
                return False
        return True

    def are_waypoints_clear(self, points, distanceFromObstacle=1):
        """
        returns True iff there is no obstacle is closer than distanceThreshold from any point
        note: return True conservatively, in case of doubt return False since there are multiple candidates
        assumption: all points are valid coordinates
        """
        # print(f"points: {points}")
        for x,y in points:
            x, y = min(x, self.m-1), min(y, self.n-1)
            # print(f"x:{x}, y:{y}")
            if distance(self.margin[x][y][1], self.margin[x][y][2]) < distanceFromObstacle:
                return False
        return True

    def cornerVertex(self, coordinate):
        """
        returns all the corners of the obstacle boundary in which the coordinate is present. Each corner is such that it can be reached from the coordinate w/o hitting any obstacle.
        """
        x, y = coordinate

        if self.map[x][y]>0:
            raise Exception("obstacle at src/dest location")
        obstacleId = self.margin[x][y][0]
        candidates = []
        for i in range(self.graph.numVertices):
            if obstacleId in self.neighbouringRegions[i]:
                candidates.append(i)

        reachable = []
        for c in candidates:
            vertex = self.graph.vertex[c]
            # ensure that vertex coordinate lies within pixel grid
            vertex = (min(vertex[0], self.m-1), min(vertex[1], self.n-1))
            if self.is_path_clear((x, y), vertex):
                reachable.append(c)

        if len(reachable)==0:
            raise Exception("no valid path from src to dest")
        return reachable

    def path(self, src, dest):
        """
        returns the list of waypoints to approach in order to reach the destination from src
        """
        if not (0<=src[0]<self.m and 0<=src[1]<self.n):
            raise Exception("invalid src coordinate")
        if not (0<=dest[0]<self.m and 0<=dest[1]<self.n):
            raise Exception("invalid dest coordinate")

        if self.margin[src[0]][src[1]][0] == self.margin[dest[0]][dest[1]][0] and self.is_path_clear(src, dest):
            return [src, dest]

        # srcV are vertices that can be reached from src w/o hitting any obstacle
        srcV = self.cornerVertex(src)

        # destV are vertices that can be reached from dest w/o hitting any obstacle
        destV = self.cornerVertex(dest)

        # print(f"src = {src}, dest = {dest}")
        # print(f"srcV = {[self.graph.vertex[v] for v in srcV]}, destV = {[self.graph.vertex[v] for v in destV]}")

        # graphPath = self.graph.path(srcV, destV)
        shortestPathLength = np.inf
        shortestPath = None
        for s in srcV:
            for d in destV:
                graphPath, pathLength = self.graph.shortest_path(s, d)
                sVertex = self.graph.vertex[s]
                dVertex = self.graph.vertex[d]
                pathLength += distance(src[0]-sVertex[0], src[1]-sVertex[1]) + distance(dest[0]-dVertex[0], dest[1]-dVertex[1])
                tPath = self.graph.transform(graphPath)
                # print(self.graph.vertex[s], self.graph.vertex[d], pathLength, tPath)
                if pathLength < shortestPathLength:
                    shortestPath = graphPath
                    shortestPathLength = pathLength
        transformedPath = self.graph.transform(shortestPath)
        # each coordinate of path should be a valid pixel location
        pixelPath = [ (min(x, self.m-1), min(y, self.n-1)) for x, y in transformedPath ]
        # print(graphPath, transformedPath, pixelPath)
        return [src] + pixelPath + [dest]

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
        voronoi.print_grid()
        voronoi.print_map()
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
        ans = [src] + [(5, 5), (5, 7), (9, 8)] + [dest]
        path = voronoi.path(src, dest)
        print(f"path: {path}")
        self.assertEqual(path, ans)
        # img = voronoi.plot_regions()
        voronoi.visualise_path(path)

    def test_random(self):
        m,n = 100, 100
        numOnes = 10
        grid = generate_random_grid(m, n, numOnes)
        voronoi = Voronoi(grid)
        # voronoi.print_grid()
        # voronoi.print_map()
        # voronoi.print_margin()
        # voronoi.print_boundary()
        src = (random.randint(0, m-1), random.randint(0, n-1))
        dest = (random.randint(0, m-1), random.randint(0, n-1))
        print(f"{src} -> {dest}")
        path = voronoi.path(src, dest)
        print(f"path: {path}")
        voronoi.visualise_path(path)

    def test_path_length(self):
        grid = readgrid("grids/2.txt")
        voronoi = Voronoi(grid)
        src, dest = (51, 42), (21, 20)
        print(f"{src} -> {dest}")
        # [(51, 42), (44, 28), (41, 26), (6, 53), (0, 53), (21, 20)]
        path = voronoi.path(src, dest)
        print(f"path: {path}")
        voronoi.visualise_path(path)

if __name__ == '__main__':
    unittest.main()