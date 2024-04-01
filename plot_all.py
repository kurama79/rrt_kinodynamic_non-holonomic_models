# Mostramos el movimientos de los agentes

import numpy as np
from collections import deque
import matplotlib.pyplot as plt 
# from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches
import matplotlib.cm as cmx
import matplotlib.colors as colors
import matplotlib.animation as animation
import matplotlib as mpl

from descartes import PolygonPatch

def visualize_dynamic(robot, animated_time=10):

	history_len = int(len(robot.solutionPath) / 150)
	environment = robot.environment

	fig1, ax1 = plt.subplots()
	ax1.set_title('Mostrando animacion del robot')
	ax1.set_xlim(environment.limits[0][0], environment.limits[0][1])
	ax1.set_ylim(environment.limits[1][0], environment.limits[1][1])

	# Generamos los puntos y los trazos
	patch = []
	trace = ax1.plot([], [], '-', color='yellow', lw=1)[0]
	histories_x = deque(maxlen=history_len)
	histories_y = deque(maxlen=history_len)

	obstacles_polygon = []
	for i in environment.Obstacles:

		obstacles_polygon.append(patches.Polygon(environment.Obstacles.get(i), closed=True, color='black'))
		ax1.add_patch(obstacles_polygon[-1])

	# Configuracion incial y final
	start = robot.car_shape(robot.path[0])
	end = robot.car_shape(robot.path[-1])

	start_patch = PolygonPatch(start, fc='blue', alpha=0.8, zorder=2)
	end_patch = PolygonPatch(end, fc='green', alpha=0.8, zorder=2)
	circle_start = plt.Circle((robot.path[0][0], robot.path[0][1]), robot.wheelRad+0.3 , fc='b', alpha=0.4)
	circle_end = plt.Circle((robot.path[-1][0], robot.path[-1][1]), robot.wheelRad+0.3 , fc='g', alpha=0.4)

	ax1.add_patch(start_patch)
	ax1.add_patch(end_patch)
	ax1.add_patch(circle_start)
	ax1.add_patch(circle_end)

	# Inicio de animacion
	car = robot.car_shape(robot.solutionPath[0])
	car_patch = PolygonPatch(car, fc='red', alpha=0.8, zorder=2)

	if robot.modelCar == 'ddr':
		circle = plt.Circle((robot.path[0][0], robot.path[0][1]), robot.wheelRad+0.3 , fc='y', alpha=0.4)

	# Funciones para animacion
	def init():

		ax1.add_patch(car_patch)
		ax1.add_patch(circle)

		return []

	def animate(index):

		this_x = robot.solutionPath[index][0]
		this_y = robot.solutionPath[index][1]

		if index == 0:

			histories_x.clear()
			histories_y.clear()

		histories_x.appendleft(this_x)
		histories_y.appendleft(this_y)

		trace.set_data(histories_x, histories_y)

		# Transformacion de poligono
		theta = robot.solutionPath[index][2] - (np.pi/2)
		x = robot.solutionPath[index][0]
		y = robot.solutionPath[index][1]

		r = mpl.transforms.Affine2D().rotate(theta)
		t = mpl.transforms.Affine2D().translate(x, y)
		tra = r + t + ax1.transData

		car_patch.set_transform(tra)

		if robot.modelCar == 'ddr':
		
			circle.center = (x, y)
			return car_patch, circle, trace

		else:
			return car_patch, trace

	ani = animation.FuncAnimation(fig1, animate, init_func=init, frames=len(robot.solutionPath), interval=animated_time)
	plt.show()

def visualize_all(robot):

	# Ver como acomodar los controles....................................
	environment = robot.environment
	shapes = []

	fig, ax = plt.subplots()
	ax.set_title('Mostrando dinamica completa del robot')
	ax.set_xlim(environment.limits[0][0], environment.limits[0][1])
	ax.set_ylim(environment.limits[1][0], environment.limits[1][1])

	obstacles_polygon = []

	for i in environment.Obstacles:

		obstacles_polygon.append(patches.Polygon(environment.Obstacles.get(i), closed=True, color='black'))
		ax.add_patch(obstacles_polygon[-1])

	for node in robot.nodes.keys():

		ax.plot(node[0], node[1], '*', color='blue', lw=1)

		if robot.nodes[node]['Parent'] != None:

			ax.plot([node[0], robot.nodes[node]['Parent'][0]], [node[1], robot.nodes[node]['Parent'][1]], '-', color='green', lw=0.5)


	for point in robot.path:

		if robot.nodes[point]['Parent'] != None:
			ax.plot([point[0], robot.nodes[point]['Parent'][0]], [point[1], robot.nodes[point]['Parent'][1]], '.', color='red', lw=1.5)

	dilated1 = robot.car_shape(robot.path[0])
	dilated2 = robot.car_shape(robot.path[-1])

	patch1 = PolygonPatch(dilated1, fc='blue', alpha=0.8, zorder=2)
	
	count = 0
	for point in robot.solutionPath:

		ax.plot(point[0], point[1], '.', color='yellow', lw=1.5)
		dynamic_dilated = robot.car_shape(point)
		dynamic_patch = PolygonPatch(dynamic_dilated, fc='red', alpha=0.4, zorder=2)
		ax.add_patch(dynamic_patch)

		# if count == 15:

		# x2 = point[0]
		# y2 = point[1]
		# x1 = point[0] - 0.5*np.cos(point[2])
		# y1 = point[1] - 0.5*np.sin(point[2])
		# print(point[2])

		# direction = patches.Arrow(x2, y2, x1, y1, width=1.5)
		# ax.add_patch(direction)

			# count = 0

		# count += 1

	patch2 = PolygonPatch(dilated2, fc='green', alpha=0.8, zorder=2)

	ax.add_patch(patch1)
	ax.add_patch(patch2)

	plt.show()