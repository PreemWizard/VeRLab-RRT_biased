import numpy as np

class Goal:
    def __init__(self, x : np.ndarray, value : int):
        self.pos = x
        self.reward = value