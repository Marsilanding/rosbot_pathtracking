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
import random




fig = plt.figure()
ax = fig.add_subplot(111)

class object:
    def __init__(self,name,lim_x,lim_y):
        self.name = name
        self.lim_x = lim_x
        self.lim_y = lim_y

class branch:
    def __init__(self,coord,father,cost):
        self.coord = coord
        self.father = father
        self.cost = cost


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

		offset_x = -dim_x/2
		offset_y = offset_x

		def evaluation(ob_ev,x,y):
				if (x <= ob_ev.lim_x[1] and x >= ob_ev.lim_x[0])\
				and (y <= ob_ev.lim_y[1] and y >= ob_ev.lim_y[0]):
					return 1
				else:
					return 0

		print_list = []

		for y in range(dim_y):
			for x in range(dim_x):
					for j in range(len(object_list)):
						if evaluation(object_list[j],x +offset_x,y +offset_y) == 1:
								print_list.append([x +offset_x, y +offset_y])
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

		ax.scatter(0, 0, c='y', marker='x')

	

		######################
		# TODO: Implement RRT* #
		######################

	
		Ps = [start_cell[1],start_cell[0]]
		
		Pe = [goal_cell[1],goal_cell[0]]
			
		D = 1.0
		
		dist_0 = 9999

		# tree es un array de objetos tipo branch(coor,father,cost)
		tree = []
		# Posicion inicial de tree
		tree.append(branch(Ps,Ps,0))
		# Condiciones iniciales para finalizar el arbol y de colision
		endTree = False
		colision = False

		# vectores auxiliares
		tree_res = []
		x_tree = []
		y_tree = []
		z_tree = []
		coord_list = []
		father_list = []

		# variables que representa cada iteracion para tener una idea
		# de como va el programa
		iter = 0

		while endTree == False:
			Pr = [random.uniform(-dim_x/2, dim_x/2), random.uniform(-dim_y/2, dim_y/2)]
			while tree.count(Pr) != 0:
				Pr = [random.uniform(-dim_x/2, dim_x/2), random.uniform(-dim_y/2, dim_y/2)]

			for point in tree:
				dist_1 = math.sqrt((Pr[0]-point.coord[0])**2 +(Pr[1]-point.coord[1])**2)
				if dist_1 < dist_0:
					Pi = point.coord
					dist_0 = dist_1

			Vd = [(Pr[0]-Pi[0])*(D/dist_0), (Pr[1]-Pi[1])*(D/dist_0)]
			Pd = [Vd[0]+Pi[0], Vd[1]+Pi[1]]    

			for j in range(len(object_list)):
				if evaluation(object_list[j], Pd[0], Pd[1]) == 1:
					colision = True
					break

			if colision == False:
				tree.append(branch(Pd, Pi, dist_0))

				dist_2 = math.sqrt((Pe[0]-Pd[0])**2 +(Pe[1]-Pd[1])**2)
				if dist_2 <= D:
					tree.append(branch(Pe, Pd, dist_2))
					endTree = True
			else:
				colision = False
		
			dist_0 = 9999
			iter = iter + 1
			print("Iteracion numero: ", iter)

		print("FIN SEARCH")
		# Hemos terminado de recopilar los puntos en tree
		# ahora hace falta ordenarlos

		for point in tree:
			coord_list.append(point.coord)
			father_list.append(point.father)
		

		s_point = Pe
		while True:
			tree_res.append(s_point)
			if s_point == Ps:
				break
			s_point = father_list[coord_list.index(s_point)]

		tree_res.reverse()

		for point in tree_res:
			x_tree.append(point[0])
			y_tree.append(point[1])

		ax.plot(x_tree, y_tree, c='b', marker='x')

		print("Arbol sin suavizar: ")
		print(tree_res)
		# tree_res representa el arbol tree con solo los puntos de interes
		# y ya organizado de inicio a final

		############## SUAVIZADO ##############

		# Esta funcion se encarga de revisar de si hay un objeto entre dos puntos dados
		# evaluando cada 0.1 en distancia entre ellos

		step = 0.1
		def rute_free(start, goal):
			m = math.sqrt((goal[0]-start[0])**2 +(goal[1]-start[1])**2)
			result = True
			for s in np.arange(step, m, step):
				v = [(goal[0]-start[0])*(s/m), (goal[1]-start[1])*(s/m)]
				p = [v[0]+start[0], v[1]+start[1]]
				for j in range(len(object_list)):
					if evaluation(object_list[j],p[0], p[1]) == 1:
						result = False
						break
			return result

		
		tree_res_s = []
		tree_res_s.append(tree_res[0])
		suav_point = tree_res[0]
		while True:
			index = tree_res.index(suav_point)
			for n in np.arange(index+1,len(tree_res)):
				prox_point = tree_res[n]
				if rute_free(suav_point, prox_point) == True:
					next_point = prox_point

		
			print("Si esto se repite sin fin reintenta de nuevo")
			print(next_point, n)

			tree_res_s.append(next_point)
			suav_point = next_point
			if next_point == tree_res[-1]:
					break

	
		######################
		# End RRT*             #
		######################

		#Print Path#

		x_tree_s = []
		y_tree_s = []
		for point in tree_res_s:
			x_tree_s.append(point[0])
			y_tree_s.append(point[1])

		ax.plot(x_tree_s, y_tree_s, c='g', marker='o')



		print("Arbol suavizado:")
		print(tree_res_s)
		print("FIN")

		plt.show()
		
		return tree_res_s

	def goto(self):
		"""Moves the robot to the goal."""
		goal_pose = Pose()
		# Get the input from the user.
		# Test with -0.5,3.5 meters
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
				
		

		current_y= (self.pose.position.y)
		current_x= (self.pose.position.x)

		current_cell = [current_y, current_x]

		goal_y= (goal_pose.position.y)
		goal_x= (goal_pose.position.x)

		goal_cell = [goal_y, goal_x]

		path = self.compute_path(current_cell,goal_cell)


        
if __name__ == '__main__':
    try:
        rospy.init_node('robot_planner', anonymous=True)

        x = Planner()
        x.goto()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
