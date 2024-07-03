import numpy as np
import matplotlib.pyplot as plt
import heapq
from py_Utils import Trajectory, CSpace


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
        path = path[::-1]
        reverse_xy = []
        for x,y in path:
            reverse_xy.append((y,x))
        return reverse_xy

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
        start = (start[1], start[0])
        goal = (goal[1], goal[0])
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
    



def main():
    map_dict = np.load('sim_map'+'.npy', allow_pickle=True)
    resolution =  map_dict.item().get('map_resolution')
    origin_x = map_dict.item().get('map_origin_x')
    origin_y = map_dict.item().get('map_origin_y')
    map_ = map_dict.item().get('map_data')
    converter = CSpace(resolution, origin_x, origin_y, map_.shape )
    show_map_mode =False
    print(converter.pixel2meter([142, 78]))
    if show_map_mode:
        plt.imshow(map_, origin="lower")
        start=converter.meter2pixel([0.0,0.0])
        goal = converter.meter2pixel([3.37, 2.05])
        plt.scatter(start[0] , start[1], c='g')
        plt.scatter(goal[0] , goal[1], c='r')
        plt.show()


    astar = A_Star(map_, inflation=int(0.4/resolution))
    start = converter.meter2pixel([-1.47, 0.13])
    goal=converter.meter2pixel([0.0, 0.0])
    path_index = astar.find_path(start, goal)
    path_meter = np.array(converter.pathindex2pathmeter(path_index))
    trajectory = Trajectory(dl=0.5, path=path_index, TARGET_SPEED=1.0)
    # np.save('path_race4_section3', path_meter)
    plt.scatter(trajectory.cx, trajectory.cy, c= "r", s=10)
    plt.imshow(map_, origin="lower")
    plt.axis('equal')
    for x,y in path_index:
        plt.scatter(x,y)
    plt.scatter(start[0] , start[1])
    plt.scatter(goal[0] , goal[1])
    plt.show()


if __name__ == "__main__":
    main()


