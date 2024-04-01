# Clase para Robot modelo DDR

import shapely.geometry
import numpy as np

from kinodynamic import RRTKinodyn
from environment import distance2Points

class DDR(object):
	def __init__(self, init_position, goal_position, direction, environment):
		
		# Parametros externos
		self.init_position = init_position
		self.goal_position = goal_position
		self.theta = direction # Angulo de orientacion del robot
		self.environment = environment
		
		# Parametros internos
		self.current_position = init_position
		self.wheelRad = 1.5
		self.robotLen = 1.5
		self.omega_left = 0 # Velocidades angulares
		self.omeha_right = 0
		self.acc_left = 0
		self.acc_right = 0
		self.acc_max = 1.8
		self.modelCar = 'ddr'

	def car_shape(self, position):

		robotRad = self.wheelRad + 0.2 # Radios del robot

		# self.shape = shapely.geometry.Point(position[0], position[1]).buffer(robotRad)
		p1 = [position[0] + robotRad*np.cos(position[2]), position[1] + robotRad*np.sin(position[2])]
		p2 = [position[0] + robotRad*np.cos(position[2]-(np.pi/2)), position[1] + robotRad*np.sin(position[2]-(np.pi/2))]
		p3 = [position[0] + robotRad*np.cos(position[2]+(np.pi/2)), position[1] + robotRad*np.sin(position[2]+(np.pi/2))]
		self.shape = shapely.geometry.Polygon([(p1[0], p1[1]), (p2[0], p2[1]), (p3[0], p3[1])])

		return self.shape

	def show_map(self):

		shape = self.car_shape(self.current_position)

		self.environment.showMap(shape)

	def generate_trayectory(self):

		# Inicializando RRT
		self.rrt = RRTKinodyn(self.environment, self.init_position, self.goal_position, 
							  [self.acc_right, self.acc_left], self.acc_max, self.wheelRad, 'ddr')

		self.path, self.solutionPath, self.nodes, self.controls = self.rrt.search()
		print(self.solutionPath)
		# input()