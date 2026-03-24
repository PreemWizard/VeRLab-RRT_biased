import numpy as np

class Tree:
    def __init__(self, root : np.ndarray):
        self.vertexes = [root]
        self.edges = []

    def add_vertex(self, vertex : np.ndarray):
        self.vertexes.append(vertex)

    def add_edge(self, v_near : np.ndarray, v_new : np.ndarray):
        self.edges.append(np.array([v_near, v_new]))