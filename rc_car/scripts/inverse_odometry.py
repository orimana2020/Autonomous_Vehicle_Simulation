import numpy as np
import matplotlib.pyplot as plt

x_s, y_s, theta_s = -2,2, np.pi/3

x_g, y_g, theta_g = 4,2, -np.pi/6

x_c = (-y_s + y_g -x_s/theta_s +x_g/ theta_g) / (-1/theta_s + 1/theta_g)
y_c = y_s -(x_c - x_s)/theta_s



fig = plt.figure()
ax = fig.add_subplot()

x_coords = []
y_coords = []
ax.scatter(x_s,y_s,c='green')
ax.arrow(x_s,y_s,  np.cos(theta_s),np.sin(theta_s))
ax.scatter(x_g,y_g,c='red')
ax.arrow(x_g,y_g, np.cos(theta_g),np.sin(theta_g))
ax.scatter(x_c,y_c, c='blue')
ax.plot([x_s,x_c],[y_s,y_c])
ax.plot([x_g,x_c],[y_g,y_c])




ax.set_aspect('equal')


plt.show()