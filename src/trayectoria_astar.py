#!/usr/bin/env python
import rospy
import math
from math import pow, atan2, sqrt
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from uav_abstraction_layer.srv import GoToWaypoint, TakeOff
import tf.transformations
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111)

class object:
    def __init__(self,name,lim_x,lim_y):
        self.name = name
        self.lim_x = lim_x
        self.lim_y = lim_y


class Planner:

	def __init__(self):
		self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
		self.pose_stamped = PoseStamped() 
		self.pose_cov = PoseWithCovariance()
		self.pose = Pose()
	
        
	def update_pose(self, data):
		"""Callback function which is called when a new message of type Pose is
		received by the subscriber."""     
		self.pose_stamped.pose = data.pose
		self.pose_cov = self.pose_stamped.pose
		self.pose = self.pose_cov.pose
		self.pose.position.x = round(self.pose.position.x, 4)
		self.pose.position.y = round(self.pose.position.y, 4)
	
        
		orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
       

	def compute_path(self, start_cell, goal_cell):
		"""Compute path."""
		path = []
		
		ob1 = object("Obstacle1",[-3, 5],[-8, -7])
		ob2 = object("Obstacle2",[9, 12],[0, 10])
		ob3 = object("Obstacle3",[-4, 0],[4, 8])

	
		object_list = [ob1, ob2, ob3]


		dim_x = 40
		dim_y = 40

		gen_map = np.zeros((dim_x, dim_y))
		print_list = []

		offset_x = -dim_x/2
		offset_y = offset_x

		def evaluation(ob_ev,x,y):
			if (x +offset_x <= ob_ev.lim_x[1] and x +offset_x >= ob_ev.lim_x[0])\
			and (y +offset_y <= ob_ev.lim_y[1] and y +offset_y >= ob_ev.lim_y[0]):
				return 1
			else:
				return 0

		
		for y in range(dim_y):
			for x in range(dim_x):
				for j in range(len(object_list)):
					if evaluation(object_list[j],x,y) == 1:
						gen_map[x,y] = 1
						print_list.append([x + offset_x, y + offset_y])
						break

		X = []
		Y = []

		for n in range(len(print_list)):
			X.append(print_list[n][0])
			Y.append(print_list[n][1])

		ax.scatter(X, Y, c='r', marker='o')

		ax.set_xlabel('X Label')
		ax.set_ylabel('Y Label')

		for element in object_list:
			ax.text(element.lim_x[0],element.lim_y[0],element.name,color='blue')

		ax.scatter(0, 0, 0, c='y', marker='x')

        ######################
        # 		   A*        #
        ######################
		class cell ():
			def __init__(self, x, y, gn, hn, fn, parent) :
				self.x = x
				self.y = y
				self.gn = gn
				self.hn = hn
				self.fn = fn
				self.parent = parent

			def calculafn(self) :
				self.hn = abs(goal_cell[1] - self.x) + abs(goal_cell[0] - self.y)	#Manhattan
				self.fn = self.gn + self.hn

	
		lista_abierta = []
		lista_cerrada = []

		#Paso 0: incluyo celda origen a la lista abierta
		celda = cell(start_cell[1], start_cell[0], 0, 0, 0, None)
		celda.calculafn()
		lista_abierta.append(celda)
	
	
		#Paso 1: ordeno lista abierta, saco primer elemento y lo meto en la cerrada 

		lista_abierta.sort(key=lambda x: x.fn)

		
		while (lista_abierta != False) :	#solo mientras queden elementos en la lista abierta
			celda = lista_abierta.pop(0)
			lista_cerrada.append(celda)
			if (celda.x == goal_cell[1] and celda.y == goal_cell[0]):	#Paso 2: he encontrado la meta, salgo
				break
					
		
			#Paso 3: calculo vecinos
			neighbors_list = []
			neighbors_list.append([celda.y + 1, celda.x])		
			neighbors_list.append([celda.y, celda.x + 1])
			neighbors_list.append([celda.y - 1, celda.x])
			neighbors_list.append([celda.y, celda.x - 1])
			

			#Paso 4: compruebo si existe en alguna lista
			for vecino in neighbors_list :		
				if (gen_map[vecino[1],vecino[0]] == 0) :	#solo busco si la celda esta libre de obstaculos 
					found = 0
					for objeto in lista_abierta : #si esta en la abierta, actualizo gn y reordeno
						if(objeto.x == vecino[1] and objeto.y == vecino[0]) :
							found = 1
							g_n = celda.gn + 1
							if (g_n < objeto.gn) :
								objeto.gn = g_n
								objeto.calculafn()
								objeto.parent = celda
								lista_abierta.sort(key=lambda x: x.fn) #reordeno la lista
							
							
					for objeto in lista_cerrada : #si esta en la cerrada, hago lo mismo
						if(objeto.x == vecino[1] and objeto.y == vecino[0]) :
							found = 1
							g_n = celda.gn + 1
							if (g_n < objeto.gn) :
								objeto.gn = g_n
								objeto.calculafn()
								objeto.parent = celda
								lista_abierta.sort(key=lambda x: x.fn) #reordeno la lista
			
					if(found==0) :			#si no esta en ninguna lista, lo incluyo y reordeno
						neighbor = cell(vecino[1],vecino[0], celda.gn + 1, 0, 0, celda)
						neighbor.calculafn()
						lista_abierta.append(neighbor)
						lista_abierta.sort(key=lambda x: x.fn)

		while(celda.parent != None):			#Una vez encontrada la meta, obtengo el camino
			path.append([celda.y, celda.x])
			celda = celda.parent

		path.append(start_cell)
		path.reverse()
	
	
        ######################
        # End A*             #
        ######################

        # Print path
		x = []
		y = []
			
		for point in path:
			x.append(point[1]-20)
			y.append(point[0]-20)
			print(point[1]-20, point[0]-20)

		ax.scatter(x, y, c='g', marker='x')
		plt.show()
			
			
		return path


	def goto(self):
		"""Moves the robot to the goal."""

		goal_pose = Pose()
        
		while True:
			data = input("Set your x goal: ")
			if data <= 20 and data >= -20:
				goal_pose.position.x = data
				break
			else:
				print("x goal must be between -20, 20")

		while True:
			data = input("Set your y goal: ")
			if data <= 20 and data >= -20:
				goal_pose.position.y = data
				break
			else:
				print("y goal must be between -20, 20")


		current_y= math.trunc(self.pose.position.y)
		current_x= math.trunc(self.pose.position.x)

		current_cell = [current_y+20, current_x+20]

		goal_y= math.trunc(goal_pose.position.y)
		goal_x= math.trunc(goal_pose.position.x)

		goal_cell = [goal_y+20, goal_x+20]

		path = self.compute_path(current_cell,goal_cell)
      
if __name__ == '__main__':
	try:
		rospy.init_node('robot_planner', anonymous=True)
		x = Planner()
		x.goto()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
