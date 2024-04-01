# Clase para Robot modelo Car-Like

import shapely.geometry
import numpy as np

from kinodynamic import RRTKinodyn
from environment import distance2Points

class car_like(object):
	def __init__(self, init_position, goal_position, direction, environment):

		# Parametros externos
		self.init_position = init_position
		self.goal_position = goal_position
		self.theta = direction # Angulo de orientacion del robot
		self.environment = environment
		
		# Parametros internos
		self.current_position = init_position
		self.robotLen = 3
		self.thickness = self.robotLen / 2 # Grosor del robot
		self.initial_vel = 0 
		self.vel_max = 2.5
		self.angular_vel = 0
		self.phi = 0 # Angulo de orientacion de las llantas (respecto al eje del robot)
		self.modelCar = 'ddr'

	def dynamic():

		return None

	def car_shape(self, position):

		x = position[0] - (self.robotLen * np.cos(position[2]))
		y = position[1] - (self.robotLen * np.sin(position[2]))

		self.shape = shapely.geometry.LineString([(x, y), (position[0], position[1])]).buffer(self.thickness, cap_style=3)

		return self.shape

	def show_map(self):

		shape = self.car_shape(self.current_position)

		self.environment.showMap(shape)

	def generate_trayectory(self):

		# Inicializando RRT
		self.rrt = RRTKinodyn(self.environment, self.init_position, self.goal_position, 
							  [self.initial_vel, self.angular_vel], self.vel_max, self.robotLen,
							  'car_like')

		self.path, self.solutionPath, self.nodes, self.controls = self.rrt.search()
		print(self.solutionPath)
		# input()