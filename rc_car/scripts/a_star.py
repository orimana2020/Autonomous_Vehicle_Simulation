import numpy as np
import matplotlib.pyplot as plt
import heapq
from scipy import interpolate


class A_Star(object):
    def __init__(self, map_: np.array, inflation=0):
        self.map = map_
        self.inflated_map = self.inflate(inflation)

    def inflate(self, inflation):#, resolution, distance):
        cells_as_obstacle = inflation #int(distance/resolution)
        original_map = self.map.copy()
        inflated_map = self.map.copy()
        self.rows, self.cols = inflated_map.shape
        for j in range(self.cols):
            for i in range(self.rows):
                if original_map[i,j] != 0:
                    i_min = max(0, i-cells_as_obstacle)
                    i_max = min(self.rows, i+cells_as_obstacle)
                    j_min = max(0, j-cells_as_obstacle)
                    j_max = min(self.cols, j+cells_as_obstacle)
                    inflated_map[i_min:i_max, j_min:j_max] = 100
        return inflated_map
    
     
        
    def h(self, current, goal):
        return ((current[0] - goal[0])**2 + (current[1] - goal[1])**2)**0.5
    
    def reconstruct_path(self, current, came_from, start):
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        return path[::-1]

    def get_neighbors(self, current, closed_list):
        '''
        return neigbors and cost.
        add only neigbors that are not in closed list
        '''
        i, j = current
        neighbors = []
        # right 
        if i < self.rows-1 and self.inflated_map[i+1,j] == 0 and closed_list[i+1,j] == 0: 
            neighbors.append([(i+1,j), 1])
        # left
        if i > 0 and self.inflated_map[i-1,j] == 0 and closed_list[i-1,j] == 0:
            neighbors.append([(i-1, j), 1])
        # up 
        if j < self.cols-1 and self.inflated_map[i,j+1] == 0 and closed_list[i,j+1] == 0:
            neighbors.append([(i,j+1), 1])
        # down
        if j > 0 and self.inflated_map[i,j-1] == 0 and closed_list[i,j-1] == 0:
            neighbors.append([(i, j-1), 1])
        return neighbors
        

    def find_path(self, start, goal):
        open_list = [(self.h(start, goal), start)]
        gscore = dict()
        came_from = dict()
        gscore[start] = 0
        closed_list = np.zeros_like(self.inflated_map, dtype=int)

        heapq.heapify(open_list)
        while open_list:
            current_cost, current = heapq.heappop(open_list)
            closed_list[current[0],current[1]] = 1
            if current == goal:
                return self.reconstruct_path(current, came_from, start)
            neighbors = self.get_neighbors(current, closed_list)
            for neighbor, distance in neighbors:
                tentative_gscore = current_cost + distance
                if neighbor not in gscore.keys():
                    gscore[neighbor] = np.inf
                if tentative_gscore < gscore[neighbor]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_gscore
                    heapq.heappush(open_list, (gscore[neighbor]+self.h(neighbor, goal), neighbor) )
        print('No Path Found')
        return None




map_ = np.array(np.load('maze_1.npy'), dtype=int)

astar = A_Star(map_, inflation=5)


# fig = plt.figure()
# ax = fig.add_subplot()
# map_ = astar.inflated_map
# map_[0:20, 0:20] = 100
# for i in range(astar.rows):
#     print(i)
    # for j in range(astar.cols):
    #     if astar.inflated_map[i][j] != 0:
    #         ax.scatter(j,i, c='black', s=120)
    #     else:
    #         ax.scatter(j,i,c='grey',s=80)

start=(78, 110)
goal=(24, 414)
path = astar.find_path(start, goal)
if path is not None:
    for vertix in path:
        map_[vertix[0], vertix[1]] = 80
x,y = [], []
for idx, vertix in enumerate(path):
    if idx % 10 ==0:
        x.append(vertix[0])
        y.append(vertix[1])

# tck_s = splrep(np.array(x), np.array(y), s=2)
# scipy.interpolate.splint(x,y,tck_s)
# out = []
# for n in range(len(x)):
#     out.append(scipy.interpolate.splint(0, x[n], tck_s))
plt.imshow(astar.inflated_map, origin="lower")
plt.axis('equal')
plt.show()



# f = interpolate.interp1d(x, y, fill_value="extrapolate", axis=0)

# ynew = f(x)   # use interpolation function returned by `interp1d`
# plt.plot(x, y, 'o', x, ynew, '-')
# plt.show()

# look_ahead_distance = 5
# WB = 0.35
        
# def pure_pursuit_steer_control(state, trajectory):
#     '''
#     Generally speaking we are given a trajectory of points in world coordinates. 
#     We also know the center point of the car in world coordinates.
#       We can measure velocity and the yaw of the car as simplified with the bicycle model.
#     '''
#     # Find the look ahead index from the trajectory
#     ind = trajectory.search_target_index(state)
    
#     # index normalization making sure we don't overshoot the goal.
#     if ind < len(trajectory.cx):
#         tx = trajectory.cx[ind]
#         ty = trajectory.cy[ind]
#     else:  # toward goal
#         tx = trajectory.cx[-1]
#         ty = trajectory.cy[-1]
#         ind = len(trajectory.cx) - 1
    
#     # Calculate the slope of look ahead distance line which would be alpha.
#     # If the car was heading along the velocity vector then that would be it but the 
#     # car has a yaw from the heading vector and hence we need to subtract this
#     # from slooe to get actual alpha
#     alpha = np.arctan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

#     # steering angle = invtan(2Lsin(alpha)/Ld)
#     # as established steering control law for pure pursuit controller
#     delta = np.arctan2(2.0 * WB * np.sin(alpha) / look_ahead_distance(state.v), 1.0)

#     return delta, ind


# def search_target_index(self, state):
#     '''
#     The function search_target_index first finds the nearest point from the rear axle on the
#       trajectory and then starting from that point calculates the index of the point on trajectory
#         which is look ahead distance away.
#       As stated earlier look ahead distance varies as a function of the velocity.
#     '''

#     def closest_index_on_trajectory():
#         dx = self.cx - state.rear_x
#         dy = self.cy - state.rear_y
#         return np.argmin(np.hypot(dx, dy))

#     def look_ahead_idx_from(closest_index):
#         target_index = closest_index
#         while look_ahead_distance(state.v) > \
#                 state.distance_from_rear_axle(self.cx[target_index], self.cy[target_index]):
#             if (target_index + 1) >= len(self.cx):
#                 break  # not exceed goal
#             target_index += 1
#         return target_index

#     return look_ahead_idx_from(closest_index_on_trajectory())