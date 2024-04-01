import numpy as np
import math
from shapely.geometry import Polygon, Point, LineString

class RRTKinodyn:

	def __init__(self, environment, initPos, goalPos, initInput, inputMax, robotLen, modelCar,
				 linear_vel=0, angle_vel=0):

		self.environment = environment
		self.initPos = (initPos[0], initPos[1], np.pi/2)
		self.goalPos = (goalPos[0], goalPos[1], np.pi/2)
		self.initInput = initInput
		self.inputMax = inputMax
		self.robotLen = robotLen
		self.modelCar = modelCar

		# Configuracion inicial
		if self.modelCar == 'ddr':
			self.initPos = (initPos[0], initPos[1], np.pi/2, linear_vel, angle_vel)
			self.goalPos = (goalPos[0], goalPos[1], np.pi/2, 0, 0)
		else:

			self.initPos = (initPos[0], initPos[1], np.pi/2)
			self.goalPos = (goalPos[0], goalPos[1], np.pi/2)

		self.height = environment.limits[1][1]
		self.width = environment.limits[0][1]

		self.vertices = {self.initPos:{'Parent': None, 'Control': initInput, 'Time': 0}}
		self.delta_t = 1.5
		self.T = 10
		# self.L = 5 # Largo del carro

	def random_state(self):

		x = int(self.width * np.random.random_sample())
		y = int(self.width * np.random.random_sample())
		theta = (2 * np.random.random_sample() - 1) * 2*np.pi

		random_state = (x, y, theta)

		return random_state

	def check_if_validSingle(self, pos):

		if pos[0]>=self.width and pos[1]>=self.height and pos[0]<0 and pos[1]<0:
			return False

		else:
			return True

	def check_if_valid(self, u, pos, currentPos):

		if self.twoPointsFree(u, currentPos) and (pos[0]<=self.width and pos[1]<=self.height and pos[0]>0 and pos[1]>0):
			return True

		else:
			return False

	def twoPointsFree(self, u, currentPos):

		x = currentPos[0]
		y = currentPos[1]
		theta = currentPos[2]
		obs = self.environment.Obstacles


		if self.modelCar == 'car_like':

			t = np.linspace(0, self.delta_t, 32)
			for dt in t:

				x = x + u[0]*np.cos(theta)*dt
				y = y + u[0]*np.sin(theta)*dt 
				theta = theta + u[1]*dt 

				collisionFree = self.isCollisionFree(obs, [x, y, theta])

				if not collisionFree:
					return collisionFree

			return collisionFree

		elif self.modelCar == 'ddr':

			b = self.robotLen
			aux = 1/(2*b)

			last_1 = aux*(aux*currentPos[3] + 0.5*currentPos[4])
			last_2 = aux*(aux*currentPos[3] - 0.5*currentPos[4])

			omega_right = last_1 + u[0]*self.delta_t
			omega_left = last_2 + u[1]*self.delta_t
			# linear_vel = currentPos[3] + (omega_left + omega_right)/2
			# angle_vel = currentPos[4] + (omega_right - omega_left)/2*b
			linear_vel = (omega_left + omega_right)/2
			angle_vel = (omega_right - omega_left)/2*b

			x = x + linear_vel*np.cos(theta)*self.delta_t
			y = y + linear_vel*np.sin(theta)*self.delta_t 
			theta = theta + angle_vel*self.delta_t 

			x_d = np.linspace(currentPos[0], x, 32)
			y_d = np.linspace(currentPos[1], y, 32)
			theta_d = np.linspace(currentPos[2], theta, 32)

			for i in range(len(x_d)):

				collisionFree = self.isCollisionFree(obs, [x_d[i], y_d[i], theta_d[i]])

				if not collisionFree:
					return collisionFree

			return collisionFree

	def isCollisionFree(self, obs, point):

		if self.modelCar == 'car_like':

			thickness = self.robotLen / 2
			x = point[0] - self.robotLen*np.cos(point[2])
			y = point[1] - self.robotLen*np.sin(point[2])

			pointToCheck = LineString([(x, y), (point[0], point[1])]).buffer(thickness, cap_style=3)

			for i in obs:

				current_obstacle = Polygon(obs.get(i))

				if pointToCheck.within(current_obstacle) or pointToCheck.intersection(current_obstacle) or pointToCheck.touches(current_obstacle):

					del current_obstacle
					del pointToCheck

					return False

			del current_obstacle
			del pointToCheck

			return True

		elif self.modelCar == 'ddr':

			robotRad = self.robotLen + 0.3
			pointToCheck = Point(point[0], point[1]).buffer(robotRad)

			for i in obs:

				current_obstacle = Polygon(obs.get(i))

				if pointToCheck.within(current_obstacle) or pointToCheck.intersection(current_obstacle) or pointToCheck.touches(current_obstacle):

					del current_obstacle
					del pointToCheck

					return False

			del current_obstacle
			del pointToCheck

			return True

	def calculate_metrics(self, state1, state2, gamma=20): # Gamma peso (importancia) para el angulo

		# Pesos (importancia) para cada parametro
		alpha = 1 
		beta = 1

		if self.modelCar == 'car_like':
			# Considera orientacion
			metrics = pow(alpha * pow((state1[0] - state2[0]), 2) + beta * pow((state1[1] - state2[1]), 2)#, 0.5)
						 + gamma * pow((state1[2] - state2[2]), 2), 0.5)

		elif self.modelCar == 'ddr':

			aux = min([abs(state1[2] - state2[2]), 2*np.pi - abs(state1[2] - state2[2])])

			metrics = pow(alpha * pow((state1[0] - state2[0]), 2) + beta * pow((state1[1] - state2[1]), 2)#, 0.5)
						 + pow(aux, 2), 0.5)

		return metrics

	def find_closest_state(self, pos):

		metrics_min = math.sqrt(self.width**2 + self.height**2)*10
		# closest = pos

		for key in self.vertices.keys():

			metrics = self.calculate_metrics(pos, key)
			if metrics < metrics_min:

				closest = key 
				metrics_min = metrics

		return closest

	def random_control(self): 

		if self.modelCar == 'car_like':

			# Editar para los modelos de carro

			v_lin = np.random.random_sample() * self.inputMax
			phi = (2 * np.random.random_sample() - 1) * np.pi/4

			u1 = v_lin * np.cos(phi)
			u2 = v_lin * (np.tan(phi)/self.robotLen)

			return [u1, u2]

		elif self.modelCar == 'ddr':

			# Controles de aceleracion

			a_left = (2*np.random.random_sample() - 1) * self.inputMax
			a_right = (2*np.random.random_sample() - 1) * self.inputMax

			return [a_right, a_left]

	def new_state(self, u, xk):

		if self.modelCar == 'car_like':

			x_new = xk[0] + u[0]*np.cos(xk[2])*self.delta_t
			y_new = xk[1] + u[0]*np.sin(xk[2])*self.delta_t
			theta_new = xk[2] + u[1]*self.delta_t

			new_state = (x_new, y_new, theta_new)

			return new_state

		elif self.modelCar == 'ddr':

			b = self.robotLen
			aux = 1/(2*b)

			last_1 = aux*(aux*xk[3] + 0.5*xk[4])
			last_2 = aux*(aux*xk[3] - 0.5*xk[4])

			omega_right = last_1 + u[0]*self.delta_t
			omega_left = last_2 + u[1]*self.delta_t

			linear_vel_new = (omega_left + omega_right)/2
			angle_vel_new = (omega_right - omega_left)/2*b

			# linear_vel_new = xk[3] + (omega_left + omega_right)/2
			# angle_vel_new = xk[4] + (omega_right - omega_left)/2*b

			constrain = 1/b*(2.5 - abs(linear_vel_new))
			if angle_vel_new > constrain:
				angle_vel_new = constrain

			x_new = xk[0] + linear_vel_new*np.cos(xk[2])*self.delta_t
			y_new = xk[1] + linear_vel_new*np.sin(xk[2])*self.delta_t
			theta_new = xk[2] + angle_vel_new*self.delta_t

			new_state = (x_new, y_new, theta_new, linear_vel_new, angle_vel_new)

			return new_state

	def complete_path(self, path, controls):

		print('Longitud de nodos', len(path), '\nLongitud de controles', len(controls))
		print(controls)
		# input()

		solutionPath = [path[0]]
		dt = self.delta_t / 36

		if self.modelCar == 'car_like':

			for index, node in enumerate(path[0:-2]):

				# Otro metodo
				x_1 = node[0]
				y_1 = node[1]
				angle_1 = node[2]

				x_2 = x_1 + controls[index][0]*np.cos(angle_1)*self.delta_t 
				y_2 = y_1 + controls[index][0]*np.sin(angle_1)*self.delta_t 
				angle_2 = angle_1 + controls[index][1]*self.delta_t 

				x_path = np.linspace(x_1, x_2, 21)
				y_path = np.linspace(y_1, y_2, 21)
				angle_path = np.linspace(angle_1, angle_2, 21)

				for i in range(len(x_path)):
					solutionPath.append((x_path[i], y_path[i], angle_path[i]))

				# Por controles..........................................................................
				# x = node[0]
				# y = node[1]
				# angle = node[2]
				# control = controls[index]

				# t = 0.0
				# while t <= self.delta_t:

				# 	x = x + control[0]*np.cos(angle)*dt
				# 	y = y + control[0]*np.sin(angle)*dt
				# 	angle = angle + control[1]*dt

				# 	solutionPath.append((x, y, angle))

				# 	t += dt

		elif self.modelCar == 'ddr':

			b = self.robotLen
			aux = 1/(2*b)

			for index, node in enumerate(path[0:-1]):

				x_1 = node[0]
				y_1 = node[1]
				angle_1 = node[2]
				lin_vel_1 = node[3]
				ang_vel_1 = node[4]

				last_1 = aux*(aux*lin_vel_1 + 0.5*ang_vel_1)
				last_2 = aux*(aux*lin_vel_1 - 0.5*ang_vel_1)

				omega_right = last_1 + controls[index][0]*self.delta_t
				omega_left = last_2 + controls[index][1]*self.delta_t

				lin_vel_2 = (omega_left + omega_right)/2
				ang_vel_2 = (omega_left - omega_right)/2*b
				x_2 = x_1 + lin_vel_2*np.cos(angle_1)*self.delta_t
				y_2 = y_1 + lin_vel_2*np.sin(angle_1)*self.delta_t
				angle_2 = angle_1 + ang_vel_2*self.delta_t

				x_path = np.linspace(x_1, x_2, 36)
				y_path = np.linspace(y_1, y_2, 36)
				angle_path = np.linspace(angle_1, angle_2, 36)
				lin_vel_path = np.linspace(lin_vel_1, lin_vel_2, 36)
				ang_vel_path = np.linspace(ang_vel_1, ang_vel_2, 36)

				t = 0.0
				angle = angle_1
				omega_right_aux = last_1
				omega_left_aux = last_2
				control = controls[index]
				count = 0
				ang = ang_vel_1
				aux_angle = abs(angle_1 - angle_2)
				while t <= self.delta_t:

					omega_right_aux = omega_right_aux + control[0]*dt
					omega_left_aux = omega_left_aux + control[1]*dt
					ang = (omega_right - omega_left)/2*b
					angle = angle + ang*dt

					angle_path[count] = angle
					
					if abs(angle - angle_1) >= abs(aux_angle):
						count += 1
						break

					count += 1
					t += dt

				for i in range(36-len(angle_path)):
					angle_path = np.append(angle_path, [angle_2])
					angle_path.append(angle_2)

				for i in range(len(x_path)):
					solutionPath.append((x_path[i], y_path[i], angle_path[i], lin_vel_path[i], ang_vel_path[i]))

				# Por controles.............................................................
				# x = node[0]
				# y = node[1]
				# angle = node[2]
				# vel = node[3]
				# ang = node[4]
				# control = controls[index]
				# last_1 = aux*(aux*vel + 0.5*ang)
				# last_2 = aux*(aux*vel - 0.5*ang)
				# omega_right = last_1
				# omega_left = last_2

				# t = 0.0
				# while t <= self.delta_t:

				# 	omega_right = omega_right + control[0]*dt
				# 	omega_left = omega_left + control[1]*dt
				# 	# omega_right = control[0]*dt
				# 	# omega_left = control[1]*dt
				# 	# omega_right = last_1 + control[0]*t
				# 	# omega_left = last_2 + control[1]*t
				# 	# vel = vel + (omega_right + omega_left)/2
				# 	# ang = ang + (omega_right - omega_left)/2*b
				# 	ang = (omega_right - omega_left)/2*b
				# 	vel = (omega_right + omega_left)/2
				# 	x = x + vel*np.cos(angle)*dt
				# 	y = y + vel*np.sin(angle)*dt
				# 	angle = angle + ang*dt

				# 	solutionPath.append((x, y, angle, vel, ang))

				# 	t += dt

				print(solutionPath[-1])
				print(path[index+1])
				# input()

		return solutionPath

	def extend(self, xnear, xrand):

		metrics_ref = 20
		metrics_max = self.calculate_metrics(xnear, xrand)
		current_state = xnear
		u = self.random_control()

		for iter in range(int(self.T/self.delta_t)):

			x_new = self.new_state(u, current_state)

			if self.check_if_valid(u, x_new, current_state):
				if self.calculate_metrics(x_new, xrand) < metrics_max:
					if self.calculate_metrics(x_new, xrand) <= metrics_ref or iter == self.T/self.delta_t-1:

						self.vertices[x_new] = {}
						self.vertices[x_new]['Parent'] = current_state
						# self.vertices[xnear]['Control'] = u
						self.vertices[x_new]['Control'] = u

						# Obteniendo los tiempos
						self.vertices[x_new]['Time'] = iter

						return x_new

					self.vertices[x_new] = {}
					self.vertices[x_new]['Parent'] = current_state
					# self.vertices[xnear]['Control'] = u
					self.vertices[x_new]['Control'] = u

					current_state = x_new
					metrics_max = self.calculate_metrics(current_state, xrand)

			else:
				return None

	def search(self):

		endReached = False
		startReached = False
		prox = 15
		path = []
		controls = []
		count = 0

		while not endReached:

			# closer = self.closerNode(count)
			xrand = self.random_state()
			xnear = self.find_closest_state(xrand)
			xnew = self.extend(xnear, xrand)

			if xnew is not None:

				if math.sqrt((xnew[0]-self.goalPos[0])**2 + (xnew[1]-self.goalPos[1])**2) <= prox:

					endReached = True 
					considered_node = xnew 
					path.append(considered_node)

				print(self.calculate_metrics(xnew, self.goalPos))
				# print(xnew, '\n', xnear)

			if count >= 2000:
			# if count >= 10000:
				break

			print(count)
			count += 1

		# print(self.vertices.keys())

		if not endReached: # Condicion por si no se enceuntra la meta

			# Verificar cercania de nodos a la meta
			metrics_min = math.sqrt(self.width**2 + self.height**2)*10

			for key in self.vertices.keys():

				metrics = self.calculate_metrics(self.goalPos, key)

				if metrics < metrics_min:

					closest = key
					metrics_min = metrics

			considered_node = closest
			path.append(considered_node)
			controls.append(self.vertices[considered_node]['Control'])

			print('Imprimiendo el mas cercano', closest)
			input()

		print('Imprimiendo el nodo considerado', considered_node)
		input()

		while not startReached:

			# print(self.vertices[considered_node]['Parent'])
			# input()

			considered_node = self.vertices[considered_node]['Parent']
			path.append(considered_node)
			controls.append(self.vertices[considered_node]['Control'])

			if self.vertices[considered_node]['Parent'] == None:
				startReached = True

		# Obtener el camino interpolado
		path.append(self.initPos)
		path.reverse()
		controls.reverse()
		# controls.append([0, 0])

		solutionPath = self.complete_path(path, controls)

		print('Imprimiendo camino', path)
		print('Solucion completa', solutionPath)

		return path, solutionPath, self.vertices, controls