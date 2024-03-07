import numpy as np
import matplotlib.pyplot as plt 

class Odom(object):
    def __init__(self):
        self.wheelbase = 0.35
        self.x = 0
        self.y = 0
        self.theta = 0

    def random_control(self, velocity, steering, time):
        theta_dot = velocity * np.tan(steering) / self.wheelbase
        dt = 0.03
        print(theta_dot * time)
        waypoints_x = []
        waypoints_y = []
        waypoints_theta = []
        waypoints_x.append(self.x)
        waypoints_y.append(self.y)
        waypoints_theta.append(self.theta)
        for _ in range(int(time/dt)):
            self.theta += theta_dot * dt
            x_dot = velocity * np.cos(self.theta)
            y_dot = velocity * np.sin(self.theta)
            self.x += x_dot * dt
            self.y += y_dot * dt
            waypoints_x.append(self.x)
            waypoints_y.append(self.y)
            waypoints_theta.append(self.theta)
        self.x = waypoints_x[-1]
        self.y = waypoints_y[-1]
        self.theta = waypoints_theta[-1]
        return [waypoints_x, waypoints_y, waypoints_theta]


# fig = plt.figure()
# ax = fig.add_subplot()
# odom = Odom()
# velocity = np.random.uniform(2.0)
# steering = np.random.uniform(-np.pi/6, np.pi/6)
# time = np.random.uniform(0.5,2)
# print(f'velocity {velocity}, steering {steering}, time {time}')
# waypoints = odom.random_control(velocity=velocity, steering=steering, time=time)
# ax.scatter(waypoints[0], waypoints[1])
# ax.set_aspect('equal', 'box')

# plt.show()


fig = plt.figure()
ax = fig.add_subplot()
odom = Odom()
velocity = 1
steering = 0.01
time = 1
print(f'velocity {velocity}, steering {steering}, time {time}')
x_coords = []
y_coords = []
for _ in range(3):
    velocity = np.random.uniform(2.0)
    steering = np.random.uniform(-np.pi/6, np.pi/6)
    time = np.random.uniform(0.5,2)
    waypoints = odom.random_control(velocity=velocity, steering=steering, time=time)
    ax.scatter(waypoints[0], waypoints[1])
ax.set_aspect('equal', 'box')

plt.show()



        




        
