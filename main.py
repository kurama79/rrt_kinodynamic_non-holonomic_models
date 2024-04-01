'''
Tarea 2 - RRT Kinodynamic.
	1. Implementado en Car-like robot (Control velocidad)
	2. Implementado DDR (Contol aceleracion)
'''

import numpy as np

import environment
from carLikeRobot import car_like
from ddRobot import DDR
from plot_all import visualize_dynamic, visualize_all

if __name__=='__main__':
	
	# Definiendo parametros  
	limits = [[0, 100], [0, 100], [0, 0]] # Ponemos los limites de Z en 0 para despues extenderlo a 3D
	num_obs = 25
	obs_radius_range = 3.5

	# Generando el entorno...
	work_environment = environment.Map(limits, num_obs, obs_radius_range)
	work_environment.obstaclesGeneration()
	# work_environment.showMap() # Show the neviorenment 

	# Generando los modelos de carro
	init_position = [0, 0]
	goal_position = [95, 95]
	car_direction = np.pi/2
	dtLimit = 2.5

	vel_init = 0 # Velocidades
	vel_max = 2.5
	angulVel_initial = 0

	# Car-Like Robot
	# car_like_robot = car_like(init_position, goal_position, car_direction, work_environment)
	# car_like_robot.generate_trayectory()
	# visualize_all(car_like_robot)
	# visualize_dynamic(car_like_robot)

	# DDR
	dd_robot = DDR(init_position, goal_position, car_direction, work_environment)
	dd_robot.generate_trayectory()
	visualize_all(dd_robot)
	visualize_dynamic(dd_robot)