import numpy as np
import matplotlib.pyplot as plt
from py_Utils import  CSpace, inflate
import heapq
import time
plt.ion()

class PRM(object):
    def __init__(self, env_map,  max_itr=1200, dist=20 ):
        self.max_itr = max_itr
        self.map = inflate(env_map, 4)
        self.env_rows, self.env_cols = env_map.shape #rows~y, cols~x
        self.max_dist=dist
        self.build_prm_graph()


    def build_prm_graph(self):
        # sampling
        self.graph = {}
        for _ in range(self.max_itr):
            randon_vertex = tuple(self.sample()) 
            if not self.is_in_collision(randon_vertex):
                self.graph[randon_vertex] = []
        
        # wiring k nearest neighbors
        for vertex1 in self.graph.keys():
            self.graph[vertex1] = self.find_neighbors_in_range(vertex1)
        

    def find_neighbors_in_range(self, vertex1):
        distances = [] # tuples of [(nei, dist)]
        for vertex2 in self.graph.keys():
            if vertex1 != vertex2:
                dist=self.get_cost(vertex1, vertex2)
                if dist < self.max_dist:
                    if self.local_planner(vertex1, vertex2, dist):
                        distances.append((vertex2, dist))
        return distances
    
    def get_cost(self, vertex1, vertex2):
        return ((vertex1[0]-vertex2[0])**2+(vertex1[1]-vertex2[1])**2)**0.5
    
    def sample(self):
        return np.array([np.random.randint(self.env_cols), np.random.randint(self.env_rows)], dtype=int) # cols~x, rows~y, 
    

    def is_in_collision(self, config):
        if self.map[config[1]][config[0]] == 0:
            return False
        return True
    
    def local_planner(self, config1, config2, dist):
        configs = np.unique(np.linspace(config1, config2, num=max(3,int(dist)) ,dtype=int), axis=0)
        for config in configs:
            if self.is_in_collision(config):
                return False
        return True

class A_Star():
    def __init__(self, prm: PRM):
        self.prm = prm

    def h(self, current, goal):
        return ((current[0] - goal[0])**2 + (current[1] - goal[1])**2)**0.5

    def find_path(self, start, goal):
        start = (start[0], start[1])
        goal = (goal[0], goal[1])
        self.prm.graph[start] = self.prm.find_neighbors_in_range(start)
        for nei, dist in self.prm.graph[start]:
            self.prm.graph[nei].append((start, dist))
        self.prm.graph[goal] = self.prm.find_neighbors_in_range(goal)
        for nei, dist in self.prm.graph[goal]:
            self.prm.graph[nei].append((goal, dist))
        open_list = [(self.h(start, goal), start)]
        gscore = dict()
        came_from = dict()
        gscore[start] = 0
        closed_list = np.zeros_like(self.prm.map, dtype=int)
        heapq.heapify(open_list)
        while open_list:
            current_cost, current = heapq.heappop(open_list)
            closed_list[current[1],current[0]] = 1
            if current == goal:
                return self.reconstruct_path(current, came_from, start), current_cost
            neighbors = self.prm.graph[current]
            for neighbor, distance in neighbors:
                if closed_list[neighbor[1],neighbor[0]] ==0 : # neighbor not in closed list
                    tentative_gscore = current_cost + distance
                    if neighbor not in gscore.keys(): 
                        gscore[neighbor] = np.inf
                    if tentative_gscore < gscore[neighbor]: # add to open list or update open list
                        came_from[neighbor] = current
                        gscore[neighbor] = tentative_gscore
                        heapq.heappush(open_list, (gscore[neighbor]+self.h(neighbor, goal), neighbor) )
        print('No Path Found')
        return None, None

    def reconstruct_path(self, current, came_from, start):
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        return path[::-1]
        

class Plotter():
    def __init__(self, inflated_map): 
        self.env_rows, self.env_cols = inflated_map.shape
        self.map = inflated_map
    
    def draw_graph(self, graph, start, goal,path=None):
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        
        plt.xlim([0, self.env_cols])
        plt.ylim([0, self.env_rows])
        for vertex in graph.keys():
            plt.scatter(vertex[0], vertex[1], s=20, c='g')
        for vertex in graph.keys():
            for nei, dist in graph[vertex]:
                plt.plot([vertex[0],nei[0]], [vertex[1], nei[1]], color='b', linewidth=1)
        if path is not None:
            for i in range(len(path)-1):
                plt.plot([path[i][0],path[i+1][0]], [path[i][1],path[i+1][1]], color='r', linewidth=3)
        plt.scatter(start[0], start[1], s=100, c='g')
        plt.scatter(goal[0],goal[1], s=100, c='r')
        plt.imshow(self.map, origin="lower")
        plt.pause(100)
    


     



def main():
    start_time = time.time()
    map_dict = np.load('sim_map'+'.npy', allow_pickle=True)
    resolution =  map_dict.item().get('map_resolution')
    origin_x = map_dict.item().get('map_origin_x')
    origin_y = map_dict.item().get('map_origin_y')
    map_original = map_dict.item().get('map_data')
    converter = CSpace(resolution, origin_x=origin_x, origin_y=origin_y, map_shape=map_original.shape )
    
    prm = PRM(env_map=map_original,  max_itr=1500, dist = 30)
    astar = A_Star(prm)
    start=converter.meter2pixel([0.0,0.0])
    goal = converter.meter2pixel([-2, 0])
    print(start)
    print(goal)
    path_index, cost = astar.find_path(start, goal)
    if path_index is not None:
        np.save('path2_meter', converter.pathindex2pathmeter(path_index))    
    print(f'path cost: {cost}, time: {time.time()-start_time}')
    plotter = Plotter(prm.map)
    plotter.draw_graph(prm.graph, start, goal,path_index)


if __name__ == "__main__":
    main()


