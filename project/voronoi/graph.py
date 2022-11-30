import unittest
from collections import deque

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
            return []
        else:
            path = []
            while parent[node]!=node:
                path.append(node)
                node = parent[node]
            path.append(node)
            return path[::-1]

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

        path = graph.path(2, 8)
        print(path)
        self.assertEqual(path, [])

if __name__ == '__main__':
    unittest.main()