import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import QVector3D
import numpy as np

class Visualization(QtWidgets.QMainWindow):
	def __init__(self, obstacles, goals):
		super().__init__()

		self.graph = gl.GLViewWidget()
		self.setCentralWidget(self.graph)
		self.graph.setWindowTitle('RRT')
		
		self.resize(1024, 768)
		pos = goals[0].pos
		self.graph.setCameraPosition(pos=QVector3D(pos[0], pos[1], pos[2]), distance=100)

		self.tree_edges = gl.GLLinePlotItem(mode='lines', width=1, color='g')
		self.tree_vertexes = gl.GLScatterPlotItem(size=3, color=(1, 1, 1, 1), pxMode=True)
		self.graph.addItem(self.tree_edges)
		self.graph.addItem(self.tree_vertexes)

		self.draw_obstacles_goals(obstacles, goals)

	def draw_obstacles_goals(self, obstacles, goals):
		verts_cube = np.array([
			[-0.1, -0.1, -0.1], [-0.1, -0.1, 0.1], [-0.1, 0.1, -0.1], [-0.1, 0.1, 0.1],
			[0.1, -0.1, -0.1], [0.1, -0.1, 0.1], [0.1, 0.1, -0.1], [0.1, 0.1, 0.1]
			], dtype = float)

		faces_cube = np.array([
			[0, 3, 1], [0, 3, 2], [2, 6, 3], [2, 7, 3], [6, 4, 5], [6, 5, 7],
			[4, 0, 1], [4, 1, 5], [3, 7, 5], [3, 5, 1], [2, 6, 4], [2, 4, 0]
			])

		dt = gl.MeshData(vertexes=verts_cube, faces=faces_cube)

		for obs in obstacles:
			obstacle = gl.GLMeshItem(meshdata=dt, color=(1, 0, 0, 1), smooth=False, shader=None, drawEdges=False)
			obstacle.translate(*obs.center)
			obstacle.scale(*(obs.size / 0.2))
			self.graph.addItem(obstacle)

		for g in goals:
			goal = gl.GLMeshItem(meshdata=dt, color=(0, 0, 1, 1), smooth=False, shader=None, drawEdges=False, edgeColor=(1, 1, 1, 1))
			goal.translate(*g.pos)
			goal.scale(*(np.array([2.5, 2.5, 2.5]) / 0.2))
			self.graph.addItem(goal)

			if g == goals[0]:
				text = gl.GLTextItem(text="START", pos=g.pos)
				self.graph.addItem(text)
			else:
				text = gl.GLTextItem(text=str(g.reward), pos=g.pos)
				self.graph.addItem(text)

	def update(self, trees):
		t_edges = []
		t_verts = [] 
		
		for t in trees:
			for e in t.edges:
				t_edges.append(e[0])
				t_edges.append(e[1])

			for v in t.vertexes:
				t_verts.append(v)
		
		if t_edges:
			self.tree_edges.setData(pos=np.array(t_edges))	
		
		if t_verts:
			self.tree_vertexes.setData(pos=np.asarray(t_verts, dtype=np.float32))

		# for obs in obstacles:
        #     # Calculamos os cantos matemáticos reais
        #     # Como size é (5,5) e center é o meio, center-2.5 até center+2.5 dá o 5x5.
        #     # Se você quer que a caixa visual tenha 10x10 independente do 'size' do objeto:
		# 	w, h = 10, 10 
		# 	x0 = obs.center[0] - w/2
		# 	y0 = obs.center[1] - h/2
            
        #     # Criamos um retângulo que é um item fixo no sistema de coordenadas
		# 	rect = QtWidgets.QGraphicsRectItem(x0, y0, w, h)
		# 	rect.setPen(pg.mkPen('r'))
		# 	rect.setBrush(pg.mkBrush('r'))
		# 	self.graph.addItem(rect)

		# for goal in goals:
		# 	w_g, h_g = 5, 5 # Tamanho um pouco menor para diferenciar de obstáculos
		# 	gx0 = float(goal.pos[0]) - w_g/2
		# 	gy0 = float(goal.pos[1]) - h_g/2
			
		# 	# Cria a caixinha roxa vazia
		# 	goal_rect = QtWidgets.QGraphicsRectItem(gx0, gy0, w_g, h_g)
		# 	goal_rect.setPen(pg.mkPen(color=(255, 255, 255), width=2)) # Roxo
		# 	goal_rect.setBrush(pg.mkBrush(None)) # Fundo vazio/transparente
		# 	self.graph.addItem(goal_rect)
			
		# 	# Adiciona o valor do Goal (Recompensa) dentro da caixa
			
		# 	if goal == goals[0]:
		# 		text = pg.TextItem(text="START", color=(255, 255, 255), anchor=(0.5, 0.5))
		# 	else:
		# 		text = pg.TextItem(text=str(goal.reward), color=(255, 255, 255), anchor=(0.5, 0.5))
			
		# 	font = QtGui.QFont()
		# 	font.setPixelSize(18) # PixelSize é mais estável que PointSize em gráficos 2D
		# 	# font.setBold(True)  # Opcional: Deixar em negrito ajuda na leitura
		# 	text.setFont(font)
		# 	text.setPos(float(goal.pos[0]), float(goal.pos[1]))
		# 	self.graph.addItem(text)