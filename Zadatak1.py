import sys
from math import inf

global G
global E

class Edge:
    def __init__(self, source, destination, weight):
        self.source = source
        self.destination = destination
        self.weight = weight

class Vertex:
    def __init__(self, val):
        self.val = val
        self.neigh = []

    def add_neighbour(self, neighbour):
        self.neigh.append(neighbour)
        
    def print_neighbours(self):
        print([i.val for i in self.neigh])

def MakeGraph():
    a = Vertex("a")     # 0
    b = Vertex("b")     # 1
    c = Vertex("c")     # 2
    d = Vertex("d")     # 3
    e = Vertex("e")     # 4
    f = Vertex("f")     # 5
    g = Vertex("g")     # 6

    graph = [a, b, c, d, e, f, g]

    graph[0].add_neighbour(graph[1])    # a -> b
    graph[0].add_neighbour(graph[2])    # a -> c
    
    graph[1].add_neighbour(graph[3])    # b -> d

    graph[2].add_neighbour(graph[3])    # c -> d
    graph[2].add_neighbour(graph[4])    # c -> e

    graph[3].add_neighbour(graph[4])    # d -> e
    graph[3].add_neighbour(graph[5])    # d -> f

    graph[4].add_neighbour(graph[5])    # e -> f
    graph[4].add_neighbour(graph[6])    # e -> g

    graph[5].add_neighbour(graph[6])    # f -> g

    edges = []

    edges.append(Edge(a, b, 8))
    edges.append(Edge(a, c, 6))
    edges.append(Edge(b, d, 10))
    edges.append(Edge(c, d, 15))
    edges.append(Edge(c, e, 9))
    edges.append(Edge(d, e, 14))
    edges.append(Edge(d, f, 4))
    edges.append(Edge(e, f, 16))
    edges.append(Edge(e, g, 17))
    edges.append(Edge(f, g, 7))

    print("Neighbours of a: ")
    graph[0].print_neighbours()

    print("Neighbours of b: ")
    graph[1].print_neighbours()

    print("Neighbours of c: ")
    graph[2].print_neighbours()

    print("Neighbours of d: ")
    graph[3].print_neighbours()

    print("Neighbours of e: ")
    graph[4].print_neighbours()

    print("Neighbours of f: ")
    graph[5].print_neighbours()

    print("Neighbours of g: ")
    graph[6].print_neighbours()

    return (edges, graph)


def GetInDegrees(g, e):
    l = []
    for v in g:
        n = 0
        for edge in e:
            if edge.destination == v:
                n += 1
        l.append(n)
    return l

def GetOutDegrees(g, e):
    l = []
    for v in g:
        n = 0
        for edge in e:
            if edge.source == v:
                n += 1
        l.append(n)
    return l


def InitializeSingleSource(g, g0):
    for v in g:
        v.d = inf
        v.p = None
    g0.d = 0

def FindEdgeValue(w, u, v):
    for x in w:
        if x.source == u and x.destination == v:
            return x.weight
    return inf

def Relax(u, v, e):
    if v.d > u.d + FindEdgeValue(e, u, v):
        v.d = u.d + FindEdgeValue(e, u, v)
        v.p = u

def BellmanFord(g, e, g0):
    InitializeSingleSource(g, g0)
    for i in range(len(g)):
        for edge in e:
            Relax(edge.source, edge.destination, e)
    for edge in e:
        if edge.source.d > edge.destination.d + FindEdgeValue(e, edge.source, edge.destination):
            return False
    return True

def CreatePath(g, s, v, l):
    if v == s:
        l.append(v)
    elif v.p == None:
        return None
    else:
        CreatePath(g, s, v.p, l)
        l.append(v)
    return l

def ShortestPath(g, nodeA, nodeB, e):
    BellmanFord(g, e, nodeA)
    L = []
    L = CreatePath(G, nodeA, nodeB, L)
    n = G[len(G) - 1].d
    return (L, n)

def PrintPath(G, s, v):
    if v == s:
        print(s.val, end = " ")
    elif v.p == None:
        print("no path found from", s.val, "to", v.val, "exists")
    else:
        print_path(G, s, v.p)
        print(v.val, end = " ")


def UpdateEdge(e, nodeA, nodeB, t):
    if FindEdgeValue(e, nodeA, nodeB) != inf:
        for i in e:
            if i.source == nodeA and i.destination == nodeB:
                i.weight = t
    else:
        e.append(Edge(nodeA, nodeB, t))


def NewShortestPath():
    BellmanFord(G, E, G[0])
    L = []
    L = CreatePath(G, G[0], G[6], L)
    n = G[len(G) - 1].d
    return (L, n)


# Dijkstra
def ExtractMin(G):
    m = G[0]
    for v in G:
        if v.d < m.d:
            m = v
    G.remove(m)
    return m

def dijkstra(G, w, s):
    InitializeSingleSource(G, s)
    S = []
    Q = G[:]
    while Q:
        u = ExtractMin(Q)
        S.append(u)
        for v in G:
            Relax(u, v, w)
# Dijkstra



if __name__ == "__main__":
    print("\nZadatak 1")
    (E, G) = MakeGraph()
    print("\nNodes: ")
    print([x.val for x in G])
    print("\nEdges: ")
    print([(x.source.val, x.destination.val, x.weight) for x in E])

    print("\nZadatak 2")
    listIn = GetInDegrees(G, E)
    listOut = GetOutDegrees(G, E)
    print("In degrees: ")
    print(listIn)
    print("Out degrees: ")
    print(listOut)

    print("\nZadatak 3")
    (L, N) = ShortestPath(G, G[0], G[6], E)
    print("Shortest path from", G[0].val, "to", G[6].val, "is", N)
    for i in range(len(L)):
        print(L[i].val, end = " ")
    print()

    print("\nZadatak 4")
    print("Updated edges: ")
    UpdateEdge(E, G[0], G[1], 7)
    print([(x.source.val, x.destination.val, x.weight) for x in E])
    print("Updated old edges: ")
    UpdateEdge(E, G[0], G[1], 8)
    print([(x.source.val, x.destination.val, x.weight) for x in E])

    print("\nZadatak 5")
    print("New edge: ")
    UpdateEdge(E, G[1], G[2], -4)
    print([(x.source.val, x.destination.val, x.weight) for x in E])
    (L1, n1) = NewShortestPath()
    print("\nShortest path from", G[1].val, "to", G[2].val, "is", n1)
    for i in range(len(L1)):
        print(L1[i].val, end = " ")
    print("\n")

    print("New edge1: ")
    UpdateEdge(E, G[0], G[5], 15)
    print([(x.source.val, x.destination.val, x.weight) for x in E])
    (L1, n1) = NewShortestPath()
    print("\nShortest path from", G[1].val, "to", G[2].val, "is", n1)
    for i in range(len(L1)):
        print(L1[i].val, end = " ")
    print("\n")

    print("New edge2: ")
    UpdateEdge(E, G[5], G[6], -4)
    print([(x.source.val, x.destination.val, x.weight) for x in E])
    (L1, n1) = NewShortestPath()
    print("\nShortest path from", G[1].val, "to", G[2].val, "is", n1)
    for i in range(len(L1)):
        print(L1[i].val, end = " ")
    print("\n")

    print("New edge3: ")
    UpdateEdge(E, G[0], G[3], -2)
    UpdateEdge(E, G[3], G[4], 2)
    UpdateEdge(E, G[4], G[6], -2)
    print([(x.source.val, x.destination.val, x.weight) for x in E])
    (L1, n1) = NewShortestPath()
    print("\nShortest path from", G[1].val, "to", G[2].val, "is", n1)
    for i in range(len(L1)):
        print(L1[i].val, end = " ")
    print("\n")
	
	#s = graph[3]
	#dijkstra(graph, E, s)
	#print(x.d)