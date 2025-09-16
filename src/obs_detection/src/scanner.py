#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from src.APF_algorithm import APF
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import math


class Sub_Scan(Node):
	def __init__(self):
		super().__init__('scan_subscriber')
		self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile_sensor_data)

	def scan_callback(self, msg):
		#self.get_logger().info('Received LaserScan message:')
		#self.get_logger().info(f' Ranges: {msg.ranges}')


		obstacles = []
		# Calculate obstacle positions
		for i, range in enumerate(msg.ranges):
			if range > 4:
				continue
			print(f'range: {range}, angle: {np.rad2deg(msg.angle_min + (i * msg.angle_increment))}')
			x_pos = range * math.sin(msg.angle_min + (i * msg.angle_increment)) # + xpos of robot
			y_pos = range * math.cos(msg.angle_min + (i * msg.angle_increment)) # + xpos of robot

			if x_pos == float("inf") or x_pos == float("-inf"):
				continue
			obstacles.append([x_pos, y_pos])
			#print(f"\nOBSTACBLE AT: {obstacles[i]}")

		#print(f'i: {i} vs len(msg.ranges): {len(msg.ranges)}')

		for pos in obstacles:
			#print(f"X: {pos[0]:.3f}, Y: {pos[1]:.3f}")
			print(pos)

		# define a basic target
		target_pos = np.array([[2, 2]]) # x, y values
		# define our pathing algorithm
		alg = APF()
		alg.__init__()
		# retrieve our current position in the world
		cur_pos = np.array([[-0.5, -0.5]])

		#force = alg.getRobotForces(cur_pos, target_pos, obstacles)
		
		#alg.showPlot(obstacles, target_pos)
		x_coords = np.linspace(-2, 2, 10)
		y_coords = np.linspace(-2, 2, 10)
		z = np.zeros((10, 10))

		X,Y = np.meshgrid(x_coords, y_coords)
		#print(X)
		#print(Y)

		obstacles = np.array(obstacles)

		for i, xval in enumerate(x_coords):
			for j, yval in enumerate(y_coords):
				print(f"pos (x,y) = {xval},{yval}")
				pos = np.array([[xval, yval]])
				force = alg.getRobotForces(pos, target_pos, obstacles)
				z[i][j] = alg.prev_attractive_potential + alg.prev_repulsive_potential

		fig = plt.figure()
		ax = plt.axes(projection='3d')
		ax.plot_surface(X, Y, z, cmap='viridis', edgecolor='green')
		ax.set_title('Artificial Potential Field Generated for Robot Map')
		plt.show()

		
	def pos_callback(self, pose):
		self.get_logger().info('Received update on robot pose')

def main(args=None):
	rclpy.init(args=args)
	subscriber = Sub_Scan()

	try:
		rclpy.spin(subscriber)
	except KeyboardInterrupt:
		pass
	finally:
		subscriber.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
