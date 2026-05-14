import numpy as np

class Node:
    def __init__(self, pos: np.ndarray, parent=None):
        self.pos = pos        # Coordenadas [x, y]
        self.parent = parent  # Quem é o "pai" deste nó (outro objeto Node)
        self.cost = 0.0       # Distância acumulada desde a raiz (root)
        
        # Se houver um pai, o custo é o custo do pai + a distância até ele
        if parent is not None:
            self.cost = parent.cost + np.linalg.norm(pos - parent.pos)