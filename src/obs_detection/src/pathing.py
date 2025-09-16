from APF_algorithm import APF
import math
import numpy as np


# define a basic target
target_pos = np.array([0, -0.5]) # x, y values

# define our pathing algorithm
alg = APF()
alg.__init__()

# retrieve our current position in the world
cur_pos = np.array([[-0.5, -0.5]])

# determine distance to any obstacles using LaserScan
# determine obstacle positioning due to Scan info (r, theta)
avoid_pts = np.array([[0,0]])

# calculate our new target
force = alg.getRobotForces(cur_pos, target_pos, avoid_pts)

print(force)