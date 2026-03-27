import random as rd
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets

from tree import Tree
from viz import Visualization
from obstacle import Obstacle
from bias_roulett import Roulette
from goal import Goal

class RRT:
    def __init__(self, n_obstacles: int):
        self.trees = []
        self.n_obstacles = n_obstacles
        self.obstacles = []
        
        goal1 = Goal(np.array([50, 100]), 50)
        goal2 = Goal(np.array([50, 0]), 80)
        goal3 = Goal(np.array([100, 50]), 40)
        goal4 = Goal(np.array([0, 50]), 10)
        self.goals = [goal1, goal2, goal3, goal4]

        self.roulette = Roulette(self.goals)

    def build_RRT(self, root : np.ndarray, K, dist):
        T = Tree(root)

        for n in range(self.n_obstacles):
            valid = False
            c_rand = root.copy()
            size = (10,10)

            while not valid:
                c_rand = np.array([rd.randrange(0, 100), rd.randrange(0, 100)])
                valid = True
                if np.linalg.norm(c_rand - root) < 20:
                    valid = False
                    continue
                
                for g in self.goals:
                    if np.linalg.norm(c_rand - g.pos) < 20:
                        valid = False
                        continue

            # while np.linalg.norm(c_rand - root) < 10:
            #     c_rand = np.array([rd.randrange(0, 100), rd.randrange(0, 100)])

            obstacle = Obstacle(c_rand, size)
            self.obstacles.append(obstacle)

        for k in range(K):
            goal_pos, is_random = self.roulette.spin()
            q_rand = root.copy()

            if is_random:
                q_rand = np.array([rd.randrange(0, 100), rd.randrange(0, 100)])
            else:
                q_rand = goal_pos
                # while np.linalg.norm(q_rand - goal_pos) > 15:
                #     q_rand = np.array([rd.randrange(0, 100), rd.randrange(0, 100)])

            q_near = self.nearest_vertex(q_rand, T)
            q_new = self.new_conf(q_near, q_rand, dist)
            
            collision = False

            for obstacle in self.obstacles:
                if obstacle.check_collision(q_new):
                    collision = True
                    break 

            if not collision:
                T.add_vertex(q_new)
                T.add_edge(q_near, q_new)

        self.trees.append(T)

    def build_multiple_trees(self, K, dist):
        for go in self.goals:
            T = Tree(go.pos)

            for n in range(self.n_obstacles):
                valid = False
                c_rand = go.pos.copy()
                size = (10,10)

            while not valid:
                c_rand = np.array([rd.randrange(0, 100), rd.randrange(0, 100)])
                valid = True
                if np.linalg.norm(c_rand - root) < 20:
                    valid = False
                    continue
                
                for g in self.goals:
                    if np.linalg.norm(c_rand - g.pos) < 20 and g.pos[0] != go.pos[0]:
                        valid = False
                        continue

            # while np.linalg.norm(c_rand - root) < 10:
            #     c_rand = np.array([rd.randrange(0, 100), rd.randrange(0, 100)])

            obstacle = Obstacle(c_rand, size)
            self.obstacles.append(obstacle)

            for k in range(K):
                goal_pos, is_random = self.roulette.spin()
                q_rand = root.copy()

                if is_random or goal_pos[0] == go.pos[0]:
                    q_rand = np.array([rd.randrange(0, 100), rd.randrange(0, 100)])
                else:
                    q_rand = goal_pos
                    # while np.linalg.norm(q_rand - goal_pos) > 15:
                    #     q_rand = np.array([rd.randrange(0, 100), rd.randrange(0, 100)])

                q_near = self.nearest_vertex(q_rand, T)
                q_new = self.new_conf(q_near, q_rand, dist)
                
                collision = False

                for obstacle in self.obstacles:
                    if obstacle.check_collision(q_new):
                        collision = True
                        break 

                if not collision:
                    T.add_vertex(q_new)
                    T.add_edge(q_near, q_new)

            self.trees.append(T)


    def nearest_vertex(self, vertex : np.ndarray, tree) -> np.ndarray:
        d = np.inf
        v_near = np.array([])

        for v in tree.vertexes:
            v_dist = np.linalg.norm(vertex - v)

            if v_dist < d:
                v_near = v
                d = v_dist

        return v_near

    def new_conf(self, tree_vertex : np.ndarray, vertex_to_point : np.ndarray, distance) -> np.ndarray:
        v = np.array(vertex_to_point - tree_vertex)
        v_length = np.linalg.norm(v)
        v_unit = np.array(v / v_length)

        v_new = np.array(tree_vertex + (distance * v_unit))

        return v_new

root = np.array([50, 50])

app = QtWidgets.QApplication([])
plot = Visualization()
plot.show()
RRT = RRT(10)
# RRT.build_RRT(root, 100, 2)
RRT.build_multiple_trees(100, 2)

plot.update(RRT.trees, RRT.obstacles, RRT.goals)    

app.exec()