from random import randrange, uniform, random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from descartes import PolygonPatch

# Environment class
class Map(object):

	def __init__(self, limits, num_obs, obs_radius_range):
		self.limits = limits
		self.num_obs = num_obs
		self.obs_radius_range = obs_radius_range

		self.Obstacles = {}

	# Generacion de los obstaculos
	def obstaclesGeneration(self):

		for i in range(self.num_obs):

			vertex_number = randrange(10) + 3

			centroid = [uniform(self.limits[0][0]+13, self.limits[0][1]-13), 
						uniform(self.limits[1][0]+13, self.limits[1][1]-13)]
			vertex = []
			for j in range(vertex_number):
				
				aux = [uniform(self.limits[0][0], self.limits[0][1]), uniform(self.limits[1][0], self.limits[1][1])]
				while distance2Points(aux, centroid) > self.obs_radius_range:
					aux = [uniform(self.limits[0][0], self.limits[0][1]), uniform(self.limits[1][0], self.limits[1][1])]

				vertex.append((aux[0], aux[1]))

			if vertex:
				self.Obstacles[str(i)] = self.sortVertex(vertex)

	# Ordenando los vertices para generar poligono
	def sortVertex(self, vertex):

		centroid = np.mean(vertex, axis=0)
		vertex = np.array(vertex)
		d = vertex - centroid
		angles = np.arctan2(d[:,0], d[:,1])
		oder = vertex[np.argsort(angles)]

		for i in range(1, len(oder)-1):
			
			if np.arctan((centroid[1]-oder[i+1][1])/(centroid[0]-oder[i+1][0])) == np.arctan((centroid[1]-oder[i][1])/(centroid[0]-oder[i][0])):
				angle = np.arctan2(oder[i-1][1]-oder[i][1], oder[i-1][0]-oder[i][0])
				
				if angle == np.pi/2 or angle == -np.pi/2 or angle == np.pi or angle == -np.pi or angle == 0:
					continue
				
				else:
					
					temp = [oder[i+1][0], oder[i+1][1]]
					oder[i+1], oder[i] = oder[i], temp

		return oder

	# Mostramos el mapa con obstaculos
	def showMap(self, shape):

		fig, ax = plt.subplots()
		patch = PolygonPatch(shape, fc='red', ec='red', alpha=0.8)
		ax.add_patch(patch)
		ax.set_title('Entorno')
		ax.set_xlim(self.limits[0][0], self.limits[0][1])
		ax.set_ylim(self.limits[1][0], self.limits[1][1])

		obstacles_polygon = []

		for i in self.Obstacles:

			obstacles_polygon.append(patches.Polygon(self.Obstacles.get(i), closed=True, color='blue'))
			ax.add_patch(obstacles_polygon[-1])

		plt.show()

# Funcion para medir distancia entre dos puntos de un plano 2D
def distance2Points(p1, p2):
	return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

# Mostramos los agentes en el mapa
def plotAll(environment, agents):

	fig, ax = plt.subplots()
	ax.set_title('Mostrando el entorno con los agentes reales y virtuales')
	ax.set_xlim(environment.limits[0][0], environment.limits[0][1])
	ax.set_ylim(environment.limits[1][0], environment.limits[1][1])

	obstacles_polygon = []

	for i in environment.Obstacles:

		obstacles_polygon.append(patches.Polygon(environment.Obstacles.get(i), closed=True, color='blue'))
		ax.add_patch(obstacles_polygon[-1])

	for i in agents:
		color = (random(), random(), random(), 1)
		ax.plot(i.stepPoint[0][0], i.stepPoint[0][1], 'o', c=color)
		ax.plot(i.goal[0], i.goal[1], 'o', c=color)
		ax.plot(i.virtualAgent[0], i.virtualAgent[1], 'o', color='black')

	plt.show()