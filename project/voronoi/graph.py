import unittest
from collections import deque
import numpy as np
import heapq

class PathDoesNotExistError(Exception):
    pass

class Graph:
    def __init__(self):
        self.vertex = [] # [(x, y)] list of coordinates of vertices
        self.edges = [] # adjacency list
        self.numVertices = 0

    def add_vertex(self, coordinates):
        """
        add a graph vertex at given coordinates
        """
        # if len(coordinates)!=2:
        #     raise Exception("Coordinate is not a 2-tuple")
        self.vertex.append(coordinates)
        self.edges.append([])
        self.numVertices += 1

    def add_edge(self, a: int, b: int) -> None:
        """
        add undirected edge b/w vertices a and b
        """
        if not 0<=a<self.numVertices:
            print(f"vertex: {a}")
            raise Exception("invalid vertex a")
        if not 0<=b<self.numVertices:
            print(f"vertex: {b}")
            raise Exception("invalid vertex b")
        if a==b:
            raise Exception("self edge is not allowed")
        # if b in self.edges[a] or a in self.edges[b]:
        #     raise Exception("duplicate edge is not allowed")
        self.edges[a].append(b)
        self.edges[b].append(a)

    def path(self, src: int, dest: int):
        """
        returns a list of vertices denoting path from src to dest. path may not be shortest. path contains src and dest vertices if it exists.
        """
        if not 0<=src<self.numVertices:
            print(f"vertex: {src}")
            raise Exception("invalid src")
        if not 0<=dest<self.numVertices:
            print(f"vertex: {dest}")
            raise Exception("invalid dest")
        
        if src==dest:
            return [src]
        
        parent = [-1] * self.numVertices
        parent[src] = src
        q = deque()
        q.append(src)

        reached = False
        while q:
            node = q.popleft()
            for neighbour in self.edges[node]:
                if parent[neighbour]==-1:
                    parent[neighbour] = node
                    if neighbour==dest:
                        reached = True
                    q.append(neighbour)
            if reached:
                node = dest
                break

        if node!=dest:
            # no path exists
            raise PathDoesNotExistError() # TODO - catch in summon.py
        else:
            path = []
            while parent[node]!=node:
                path.append(node)
                node = parent[node]
            path.append(node)
            return path[::-1]

    def edge_length(self, a: int, b: int):
        """
        returns length of edge b/w vertices a and b
        """
        x1, y1 = self.vertex[a]
        x2, y2 = self.vertex[b]
        return np.sqrt( (x1-x2)**2 + (y1-y2)**2 )

    # def edge_length(self, a: int, b: int, weights):
    #     """
    #     returns length of edge b/w vertices a and b
    #     """
    #     a, b = min(a, b), max(a, b)
    #     return weights[(a, b)]

    # def shortest_path(self, src: int, dest: int, weights): # to test djikstra
    def shortest_path(self, src: int, dest: int):
        """
        returns a list of vertices denoting the shortest path from src to dest. In case of multiple shortest paths, any one is returned. path contains src and dest vertices if it exists.
        also returns distance of path
        """
        if not 0<=src<self.numVertices:
            print(f"vertex: {src}")
            raise Exception("invalid src")
        if not 0<=dest<self.numVertices:
            print(f"vertex: {dest}")
            raise Exception("invalid dest")

        if src==dest:
            return [src], 0

        parent = [-1] * self.numVertices
        parent[src] = src
        distance = [np.inf] * self.numVertices
        distance[src] = 0

        q = [(0, src)]
        visited = set()
        while q:
            currDistance, node = heapq.heappop(q)
            visited.add(node)
            if node == dest:
                break
            for neighbour in self.edges[node]:
                if neighbour in visited:
                    continue
                # newDistance = currDistance + self.edge_length(node, neighbour, weights)
                newDistance = currDistance + self.edge_length(node, neighbour)
                if distance[neighbour] == np.inf:
                    distance[neighbour] = newDistance
                    parent[neighbour] = node
                    heapq.heappush(q, (newDistance, neighbour))
                elif newDistance < distance[neighbour]:
                    q.remove((distance[neighbour], neighbour))
                    distance[neighbour] = newDistance
                    parent[neighbour] = node
                    heapq.heappush(q, (newDistance, neighbour))

        if node != dest:
            raise PathDoesNotExistError
        else:
            path = []
            while parent[node]!=node:
                path.append(node)
                node = parent[node]
            path.append(node)
            return path[::-1], distance[dest]

    def transform(self, path):
        """ 
        transforms a list of vertices into a list of coordinates
        """
        res = []
        for x in path:
            if not 0<=x<self.numVertices:
                print(f"vertex: {x}")
                raise Exception("invalid vertex")
            res.append(self.vertex[x])
        return res

class TestGraph(unittest.TestCase):
    def test_same_vertex(self):
        graph = Graph()
        n = 5
        for i in range(n):
            graph.add_vertex(i)
        for i in range(n-1):
            graph.add_edge(i, i+1)
        path = graph.path(2, 2)
        print(path)
        self.assertEqual(path, [2])

    def test_chain(self):
        graph = Graph()
        n = 50
        for i in range(n):
            graph.add_vertex(i)
        for i in range(n-1):
            graph.add_edge(i, i+1)
        path = graph.path(0, n-1)
        print(path)
        self.assertEqual(path, list(range(n)))

    def test_star(self):
        graph = Graph()
        n = 10
        for i in range(n):
            graph.add_vertex(i)
        for i in range(1, n):
            graph.add_edge(0, i)
        path = graph.path(2, n-1)
        print(path)
        self.assertEqual(path, [2, 0, n-1])

    def test_diamond(self):
        graph = Graph()
        n = 9
        for i in range(n):
            graph.add_vertex(i)
        graph.add_edge(0, 2)
        graph.add_edge(1, 2)
        graph.add_edge(3, 2)
        graph.add_edge(4, 2)
        graph.add_edge(3, 5)
        graph.add_edge(4, 5)
        graph.add_edge(6, 5)
        graph.add_edge(7, 5)
        graph.add_edge(6, 8)
        graph.add_edge(7, 8)

        path = graph.path(2, 8)
        print(path)
        self.assertEqual(path, [2, 3, 5, 6, 8])

    def test_unreachable(self):
        graph = Graph()
        n = 9
        for i in range(n):
            graph.add_vertex(i)
        graph.add_edge(0, 2)
        graph.add_edge(1, 2)
        graph.add_edge(3, 2)
        graph.add_edge(4, 2)
        graph.add_edge(3, 5)
        graph.add_edge(4, 5)
        graph.add_edge(6, 5)
        graph.add_edge(7, 5)
        # graph.add_edge(6, 8)
        # graph.add_edge(7, 8)

        exceptionRaised = False
        try:
            path = graph.path(2, 8)
        except PathDoesNotExistError:
            exceptionRaised = True
        self.assertTrue(exceptionRaised)

    def test_djikstra(self):
        # uncomment function with weights before running this test
        graph = Graph()
        n = 9
        for i in range(n):
            graph.add_vertex(i)
        graph.add_edge(0, 1)
        graph.add_edge(0, 7)
        graph.add_edge(1, 2)
        graph.add_edge(1, 7)
        graph.add_edge(2, 3)
        graph.add_edge(2, 5)
        graph.add_edge(2, 8)
        graph.add_edge(3, 4)
        graph.add_edge(3, 5)
        graph.add_edge(4, 5)
        graph.add_edge(5, 6)
        graph.add_edge(6, 7)
        graph.add_edge(6, 8)
        graph.add_edge(7, 8)
        weights = {}
        weights[(0, 1)] = 4
        weights[(0, 7)] = 8
        weights[(1, 2)] = 8
        weights[(1, 7)] = 11
        weights[(2, 3)] = 7
        weights[(2, 5)] = 4
        weights[(2, 8)] = 2
        weights[(3, 4)] = 9
        weights[(3, 5)] = 14
        weights[(4, 5)] = 10
        weights[(5, 6)] = 2
        weights[(6, 7)] = 1
        weights[(6, 8)] = 6
        weights[(7, 8)] = 7

        for i in range(n):
            path, distance = graph.shortest_path(0, i, weights)
            print(f"{0} -> {i} | {distance} | {path}")

if __name__ == '__main__':
    unittest.main()