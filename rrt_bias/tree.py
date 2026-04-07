import numpy as np
from bias_roulett import Roulette

class Tree:
    def __init__(self, root : np.ndarray, roulette : Roulette):
        self.root = root
        self.vertexes = [root]
        self.edges = []
        self.roulette = roulette
        self.tree_connected_to = []

    def add_vertex(self, vertex : np.ndarray):
        self.vertexes.append(vertex)

    def add_edge(self, v_near : np.ndarray, v_new : np.ndarray):
        self.edges.append(np.array([v_near, v_new]))

    def add_connected_tree(self, tree_root : np.ndarray):
        self.tree_connected_to.append(tuple(tree_root))