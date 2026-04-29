import random as rd
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer

from tree import Tree
from viz import Visualization
from obstacle import Obstacle
from bias_roulett import Roulette
from goal import Goal

#Anotacoes:

#Expandi, temrinei, verificar se algum vertice da arvore atual  e conectavel com outro vertice das outras arvores. Se conectar, tirar os goals das arvores das roletas que forma concecatadas
#Expandir a arvore para maximizar a recompensa dentro do budget sem ter quer conectar todas as arvores
#Criar um ponto de partida e um ponto de chegada (Pode ser o mesmo ou diferente)

<<<<<<< HEAD
#Rodar o phsical, tornar o bias dinamico (remover a fatia do bias da roleta e colocar na aleatoriedade), colocar os pontos do nosso simulador no simulador do phisical, usar o planejador de caminhos do phisical no nosso simulador 

=======
>>>>>>> 5cebe3017828c79cde23f1beb8b0434ff9c98be5
class RRT:
    def __init__(self, n_obstacles: int):
        self.trees = []
        self.n_obstacles = n_obstacles
        self.obstacles = []
        
        goal1 = Goal(np.array([50, 100]), 50)
        goal2 = Goal(np.array([50, 0]), 80)
        goal3 = Goal(np.array([100, 50]), 40)
        goal4 = Goal(np.array([0, 50]), 10)
        goal5 = Goal(np.array([0, 0]), 100)
        goal6 = Goal(np.array([0, 100]), 30)
        goal7 = Goal(np.array([100, 100]), 110)
        goal8 = Goal(np.array([100, 0]), 20)
        self.goals = [goal1, goal2, goal3, goal4, goal5, goal6, goal7, goal8]

        square1 = Obstacle(np.array([25, 25]), np.array([20, 20]))
        square2 = Obstacle(np.array([25, 75]), np.array([20, 20]))
        square3 = Obstacle(np.array([75, 25]), np.array([20, 20]))
        square4 = Obstacle(np.array([75, 75]), np.array([20, 20]))
        square5 = Obstacle(np.array([50, 50]), np.array([20, 20]))

        # square6 = Obstacle(np.array([0, 25]), np.array([20, 20]))
        # square7 = Obstacle(np.array([0, 75]), np.array([20, 20]))
        # square8 = Obstacle(np.array([25, 0]), np.array([20, 20]))
        # square9 = Obstacle(np.array([25, 100]), np.array([20, 20]))
        # square10 = Obstacle(np.array([75, 100]), np.array([20, 20]))
        # square11 = Obstacle(np.array([75, 0]), np.array([20, 20]))
        # square12 = Obstacle(np.array([100, 25]), np.array([20, 20]))
        # square13 = Obstacle(np.array([100, 75]), np.array([20, 20]))

        #square6, square7, square8, square9, square10, square11, square12, square13
        self.obstacles = [square1, square2, square3, square4, square5]

        #self.roulette = Roulette(self.goals)

    def build_RRT(self, root : np.ndarray, K, dist):
        T = Tree(root, Roulette(self.goals))

        for k in range(K):
            goal_pos, is_random = T.roulette.spin()
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
                if obstacle.check_collision(q_near, q_new):
                    collision = True
                    break 

            if not collision:
                T.add_vertex(q_new)
                T.add_edge(q_near, q_new)

        self.trees.append(T)

    def prepare_trees(self):
        """Inicializa as raízes das árvores antes de começar a expansão."""
        for go in self.goals:
            t_goals = [g for g in self.goals if g != go]
            T = Tree(go.pos, Roulette(t_goals))
            self.trees.append(T)

    def expand_step(self, dist):
        """Executa UMA rodada de expansão para todas as árvores (Equivalente a k=1)."""
        n_completed = 0
        t_list_len = len(self.trees)
        for T in self.trees:
            if T.roulette.empty:
                n_completed = n_completed+1

        if n_completed == (t_list_len):
            print("Todas as arvores conectadas")
            return True
        for T in self.trees:
            goal_pos, is_random = T.roulette.spin(T.tree_connected_to)

            if T.roulette.empty:
                print(f"Roleta da árvore {tuple(T.root)} vazia - Operando em modo 100% aleatório")
                continue

            if is_random:
                q_rand = np.array([rd.randrange(0, 100), rd.randrange(0, 100)])
            else:
                q_rand = goal_pos

            q_near = self.nearest_vertex(q_rand, T)
            q_new = self.new_conf(q_near, q_rand, dist)
            
            collision = False
            for obstacle in self.obstacles:
                if obstacle.check_collision(q_near, q_new):
                    collision = True
<<<<<<< HEAD
                    T.roulette.prob_random += 0.1
                    break

            if not collision:
                if T.roulette.prob_random > 0.0:
                    T.roulette.prob_random -= 0.1
=======
                    break

            if not collision:
>>>>>>> 5cebe3017828c79cde23f1beb8b0434ff9c98be5
                T.add_vertex(q_new)
                T.add_edge(q_near, q_new)

            for other_tree in self.trees:
                if other_tree is T: 
                    continue
                
                vert_other = self.nearest_vertex(q_new, other_tree)
                root_tuple = tuple(other_tree.root)
                
                if np.linalg.norm(vert_other - q_new) <= dist and root_tuple not in T.tree_connected_to:
                    bridge_collision = False
                    for obstacle in self.obstacles:
                        if obstacle.check_collision(vert_other, q_new):
                            bridge_collision = True
                            break
                    
                    if not bridge_collision:

                        T.add_edge(vert_other, q_new)
                        T.add_connected_tree(other_tree.root)
                        other_tree.add_connected_tree(tuple(T.root))
                        #Aqui podemos adicionar a aresta a outra arvore tambem
                        print(f"Ponte criada entre a árvore {T.root} e {other_tree.root}")
        return False



    def build_multiple_trees(self, K, dist):
        for go in self.goals:
            t_goals = [g for g in self.goals if g != go]
            T = Tree(go.pos, Roulette(t_goals))
            
            self.trees.append(T)
        
        for k in range(K):
            for T in self.trees:
                goal_pos, is_random = T.roulette.spin()
                q_rand = T.root.copy()

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
                    if obstacle.check_collision(q_near, q_new):
                        collision = True
                        break

                if not collision:
                    T.add_vertex(q_new)
                    T.add_edge(q_near, q_new)
    
    def build_obstacles(self):
        for n in range(self.n_obstacles):
                valid = False
                go = rd.choice(self.goals)
                c_rand = go.pos.copy()
                size = (10,10)

                while not valid:
                    c_rand = np.array([rd.randrange(0, 100), rd.randrange(0, 100)])
                    valid = True
                    if np.linalg.norm(c_rand - go.pos) < 20:
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
    
    # def connect_all_into_graph(self, K_dist):
    #     # Criamos uma estrutura para segurar o grafo global
    #     # Pode ser uma instância da classe Tree ou uma nova classe Graph
    #     global_graph = Tree(None, None) # Raiz arbitrária
    #     global_graph.vertexes = []
    #     global_graph.edges = []

    #     # 1. Copia todos os dados de todas as árvores para o grafo global
    #     for T in self.trees:
    #         global_graph.vertexes.extend(T.vertexes)
    #         global_graph.edges.extend(T.edges)

    #     # 2. Tenta conectar árvores diferentes
    #     # Comparamos cada árvore com as próximas na lista
    #     for i in range(len(self.trees)):
    #         for j in range(i + 1, len(self.trees)):
    #             tree_a = self.trees[i]
    #             tree_b = self.trees[j]

    #             for v_a in tree_a.vertexes:
    #                 for v_b in tree_b.vertexes:
    #                     dist = np.linalg.norm(v_a - v_b)
                        
    #                     if dist <= K_dist:
    #                         # Checa colisão antes de conectar
    #                         collision = False
    #                         for obs in self.obstacles:
    #                             if obs.check_collision(v_a, v_b):
    #                                 collision = True
    #                                 break
                            
    #                         if not collision:
    #                             global_graph.add_edge(v_a, v_b)
    #                             # Opcional: break se quiser apenas uma conexão entre árvores
    #     return global_graph

root = np.array([50, 50])

app = QtWidgets.QApplication([])
plot = Visualization()
plot.show()
RRT = RRT(10)
#RRT.build_obstacles()

#RRT.build_RRT(root, 200, 2)
#RRT.build_multiple_trees(100, 2)

#global_graph = RRT.connect_all_into_graph(2)

RRT.prepare_trees()

k_max = 150
k_atual = 0
all_conected = False

def loop_de_atualizacao():
    global k_atual
    global all_conected
    if k_atual < k_max and all_conected == False:
        
        all_conected = RRT.expand_step(2)
        
        plot.update(RRT.trees, RRT.obstacles, RRT.goals)
        
        k_atual += 1
        print(f"Iteração: {k_atual}/{k_max}")
    else:
        timer.stop()
        print("Simulação concluída!")

timer = QTimer()
timer.timeout.connect(loop_de_atualizacao)
timer.start(200)

#plot.update(RRT.trees, RRT.obstacles, RRT.goals)    

app.exec()