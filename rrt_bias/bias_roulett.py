import numpy as np
from goal import Goal

class Roulette:
    def __init__(self, real_goals: list[Goal]):
        self.real_goals = real_goals
        
        self.prob_random = 0.6 
        
        # Pesos das metas reais (80% da roleta dividida entre elas)
        self.weights = [g.reward for g in self.real_goals]
        self.total_real_reward = sum(self.weights)

    def spin(self):
        # Sorteio inicial: Caiu no slot aleatório ou nas metas reais?
        if np.random.random() < self.prob_random:
            # Retorna Flag de aleatoriedade pura
            return None, True 
        
        # Caso contrário, sorteia entre as metas reais baseado no peso (IG)
        probs = [w / self.total_real_reward for w in self.weights]
        selected = np.random.choice(self.real_goals, p=probs)
        
        return selected.pos, False