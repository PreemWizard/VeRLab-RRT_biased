import pyqtgraph as pg
from PyQt5 import QtWidgets, QtGui, QtCore
import numpy as np

class Visualization(QtWidgets.QMainWindow):
	def __init__(self):
		super().__init__()

		self.graph = pg.PlotWidget(title="RRT")
		self.setCentralWidget(self.graph)
		self.graph.resize(1000, 900)

		self.tree_vertexes = self.graph.plot([], [], pen=None, symbol='o')
		self.tree_edges = self.graph.plot([], [], pen='g')

		self.graph_vertexes = self.graph.plot([], [], pen=None, symbol='o')
		self.graph_edges = self.graph.plot([], [], pen=pg.mkPen(color='y', width=1, style=QtCore.Qt.DashLine))

		self.obstacles_plot = pg.ScatterPlotItem(pen='r', brush='r', symbol='s')
		self.graph.addItem(self.obstacles_plot)


	def update(self, trees, obstacles, goals):
		x_edge = []
		y_edge = []

		x_vert = []
		y_vert = []
		
		for t in trees:
			for e in t.edges:
				x_edge += [e[0][0], e[1][0], np.nan]
				y_edge += [e[0][1], e[1][1], np.nan]

			for v in t.vertexes:
				x_vert += [v[0]]
				y_vert += [v[1]]
			
		self.tree_vertexes.setData(x_vert, y_vert)
		self.tree_edges.setData(x_edge, y_edge)

		# global_x_edge = []
		# global_y_edge = []

		# global_x_vert = []
		# global_y_vert = []

		# for e in global_graph.edges:
		# 	global_x_edge += [e[0][0], e[1][0], np.nan]
		# 	global_y_edge += [e[0][1], e[1][1], np.nan]

		# for v in global_graph.vertexes:
		# 	global_x_vert += [v[0]]
		# 	global_y_vert += [v[1]]
			
		# self.graph_vertexes.setData(global_x_vert, global_y_vert)
		# self.graph_edges.setData(global_x_edge, global_y_edge)

		for obs in obstacles:
            # Calculamos os cantos matemáticos reais
            # Como size é (5,5) e center é o meio, center-2.5 até center+2.5 dá o 5x5.
            # Se você quer que a caixa visual tenha 10x10 independente do 'size' do objeto:
			w, h = 10, 10 
			x0 = obs.center[0] - w/2
			y0 = obs.center[1] - h/2
            
            # Criamos um retângulo que é um item fixo no sistema de coordenadas
			rect = QtWidgets.QGraphicsRectItem(x0, y0, w, h)
			rect.setPen(pg.mkPen('r'))
			rect.setBrush(pg.mkBrush('r'))
			self.graph.addItem(rect)

		for goal in goals:
			w_g, h_g = 5, 5 # Tamanho um pouco menor para diferenciar de obstáculos
			gx0 = float(goal.pos[0]) - w_g/2
			gy0 = float(goal.pos[1]) - h_g/2
			
			# Cria a caixinha roxa vazia
			goal_rect = QtWidgets.QGraphicsRectItem(gx0, gy0, w_g, h_g)
			goal_rect.setPen(pg.mkPen(color=(255, 255, 255), width=2)) # Roxo
			goal_rect.setBrush(pg.mkBrush(None)) # Fundo vazio/transparente
			self.graph.addItem(goal_rect)
			
			# Adiciona o valor do Goal (Recompensa) dentro da caixa
			
			if goal == goals[0]:
				text = pg.TextItem(text="START", color=(255, 255, 255), anchor=(0.5, 0.5))
			else:
				text = pg.TextItem(text=str(goal.reward), color=(255, 255, 255), anchor=(0.5, 0.5))
			
			font = QtGui.QFont()
			font.setPixelSize(18) # PixelSize é mais estável que PointSize em gráficos 2D
			# font.setBold(True)  # Opcional: Deixar em negrito ajuda na leitura
			text.setFont(font)
			text.setPos(float(goal.pos[0]), float(goal.pos[1]))
			self.graph.addItem(text)