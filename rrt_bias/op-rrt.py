import numpy as np
import random
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.parent = None
        self.cost = 0.0
        self.reward_ids = set()
        self.total_reward = 0

def get_dist(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

class OrienteeringRRT_Final:
    def __init__(self, start, budget, targets, obstacles, area):
        self.start_pos = start
        self.budget = budget
        self.targets = targets # (id, x, y, val)
        self.obstacles = obstacles
        self.area = area
        self.nodes = [Node(start[0], start[1])]

    def is_safe(self, p1, p2):
        for (ox, oy, r) in self.obstacles:
            for i in range(11):
                px = p1[0] + (p2[0] - p1[0]) * (i / 10)
                py = p1[1] + (p2[1] - p1[1]) * (i / 10)
                if get_dist((px, py), (ox, oy)) <= r + 0.2: return False
        return True

    def plan(self, iter=2000, step=2.0):
        best_path = []
        max_reward = -1

        for _ in range(iter):
            # Amostragem com viés nos alvos
            if random.random() < 0.4:
                t = random.choice(self.targets)
                rnd = (t[1], t[2])
            else:
                rnd = (random.uniform(0, self.area), random.uniform(0, self.area))

            # Expansão
            nearest = min(self.nodes, key=lambda n: get_dist((n.x, n.y), rnd))
            theta = np.arctan2(rnd[1] - nearest.y, rnd[0] - nearest.x)
            new_pos = (nearest.x + step * np.cos(theta), nearest.y + step * np.sin(theta))

            if self.is_safe((nearest.x, nearest.y), new_pos):
                new_node = Node(new_pos[0], new_pos[1])
                new_node.parent = nearest
                new_node.cost = nearest.cost + step
                new_node.reward_ids = nearest.reward_ids.copy()
                new_node.total_reward = nearest.total_reward

                # Check exact target visit
                for tid, tx, ty, val in self.targets:
                    if tid not in new_node.reward_ids and get_dist((new_node.x, new_node.y), (tx, ty)) < 1.0:
                        new_node.x, new_node.y = tx, ty # Snap to target
                        new_node.reward_ids.add(tid)
                        new_node.total_reward += val
                
                self.nodes.append(new_node)

                # FECHAMENTO DE CICLO: O nó atual consegue ver a base?
                if self.is_safe((new_node.x, new_node.y), self.start_pos):
                    dist_back = get_dist((new_node.x, new_node.y), self.start_pos)
                    if new_node.cost + dist_back <= self.budget:
                        if new_node.total_reward > max_reward:
                            max_reward = new_node.total_reward
                            # Constrói o caminho completo: Base -> Nó -> Base
                            path = []
                            curr = new_node
                            while curr:
                                path.append((curr.x, curr.y))
                                curr = curr.parent
                            best_path = path[::-1] # Inverte para começar na base
                            best_path.append(self.start_pos) # Adiciona volta à base

        return best_path, max_reward

    def plot(self, path, reward):
        plt.figure(figsize=(8, 8))
        for ox, oy, r in self.obstacles:
            plt.gca().add_patch(plt.Circle((ox, oy), r, color='red', alpha=0.3))
        for _, tx, ty, v in self.targets:
            plt.scatter(tx, ty, marker='*', s=200, color='gold', edgecolors='black', zorder=5)

        if path:
            px, py = zip(*path)
            plt.plot(px, py, 'g-', lw=2, label="Ciclo de Coleta")
            # Setas para indicar direção
            for i in range(len(path)-1):
                plt.annotate('', xy=path[i+1], xytext=path[i], 
                             arrowprops=dict(arrowstyle="->", color='green', lw=1))

        plt.scatter(self.start_pos[0], self.start_pos[1], c='blue', s=150, marker='X', label="Base")
        plt.title(f"Ciclo Final Orienteering RRT\nRewards: {reward} | Path: {len(path)} nós")
        plt.legend(); plt.grid(True); plt.show()

# Configuração
targets = [(1, 15, 15, 100), (2, 5, 15, 80), (3, 15, 5, 60), (4, 10, 10, 40)]
obs = [(8, 12, 2), (12, 8, 2), (8, 8, 2)]
planner = OrienteeringRRT_Final(start=(2, 2), budget=200, targets=targets, obstacles=obs, area=20)
caminho, pontos = planner.plan()
planner.plot(caminho, pontos)