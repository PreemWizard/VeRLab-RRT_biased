import numpy as np
from goal import Goal

class Roulette:
    def __init__(self, real_goals: list[Goal]):
        self.real_goals = real_goals
        
        self.prob_random = 0.0
        self.empty = False
        

    def spin(self, connected_roots_tuples: list):
        """
        Sorteia um objetivo ignorando aqueles que já estão conectados à árvore.
        """
        available_goals = [
            g for g in self.real_goals 
            if tuple(g.pos) not in connected_roots_tuples
        ]

        if not available_goals:
            self.empty = True
            return None, True
        else:
            self.empty = False 
        
        if np.random.random() < self.prob_random:
            return None, True 

        weights = [g.reward for g in available_goals]
        total_reward = sum(weights)
        probs = [w / total_reward for w in weights]
        
        selected = np.random.choice(available_goals, p=probs)
        return selected.pos, False
    