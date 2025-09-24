#!/usr/bin/python3
# Creating a system to generate an Artificial Potential field for path planning
# Author: Emily Edwards
# Last Modified: Sep 16, 2025

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from scipy.spatial.distance import cdist

class APF():
    """
    Artifical Potential Field Class
    """

    def __init__(self):
        # Set APF Parameters
        self.zeta = 5 # position gain / attraction gain
        self.eta = 0.1 # constant gain
        self.rho_0 = 0.7 # limit distance of potential field influence
        self.obstacles = []

    def defineObstacles(self, objects):
        self.obstacles = objects

        #for i, obs in enumerate(objects):
        #    self.obstacles.append(obs)
            

    def getRepulsiveForce(self, cur_pos) -> np.ndarray:
        
        obstacles = self.obstacles # they have a x and y position
        # RETRIEVE CURRENT ROBOT POSITION
        # cur_pos = np.array([[self.robot.x, self.robot.y]])

        #print(obstacles.shape)
        #print(cur_pos.shape)

        potential = 0
        force = 0

        for obs in obstacles:
            #print(obs)
            obs = np.array([obs])
            #print(obs.shape)
            rho = cdist(obs, cur_pos)
            if rho <= self.rho_0:
                potential += 0.5 * self.eta * (1/rho - 1/self.rho_0)**2
                force += self.eta * (1/self.rho_0 - 1/rho) * 1/(rho**2) * cdist(cur_pos, obs)
        """       
        # calculate repulsive potential
        for i, rad in enumerate(rho):
            if rad > self.rho_0:
                potential = 0
            else:
                potential += 0.5 * self.eta * (1/rad - 1/self.rho_0)**2
        """
        self.prev_repulsive_potential = potential

        #force = self.eta * (1/self.rho_0 - 1/rho) * 1/(rho**2) * (cur_pos - obstacles)
        print(f'\nREPULSIVE FORCE: {force}')

        return force


    def getAttractiveForce(self, cur_pos: np.ndarray, tgt_pos: np.ndarray) -> np.ndarray:

        x_d = tgt_pos
        x = cur_pos
        #print(x_d.shape)
        #print(x.shape)

        potential = 0.5 * self.zeta * cdist(x, x_d)**2
        self.prev_attractive_potential = potential

        #print('ATTRACTIVE POTENTIAL: ', potential)

        force = self.zeta * cdist(x, x_d)
        print(f'\nATTRACTIVE FORCE: {force}')
        
        return force
    
    def getRobotForces(self, cur_pos: np.ndarray, tgt_pos: np.ndarray, obstacles) -> np.ndarray:

        # ensure obstacles are recorded
        self.defineObstacles(obstacles)

        F_rep = self.getRepulsiveForce(cur_pos)
        F_att = self.getAttractiveForce(cur_pos, tgt_pos)

        F_sum = F_att + F_rep
        self.prev_F_sum = F_sum

        return F_sum
    
    def showPlot(self, obstacles, target):
        x = obstacles[0] + target[0]
        y = obstacles[1] + target[1]
        z = self.prev_attractive_potential + self.prev_repulsive_potential
        print(z)

        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot_surface(x, y, z, cmap='viridis', edgecolor='green')
        ax.set_title('Artificial Potential Field Generated for Robot Map')
        plt.show()