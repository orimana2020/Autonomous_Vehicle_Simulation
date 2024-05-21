import numpy as np
import matplotlib.pyplot as plt
from py_Utils import Tree, calc_configs_dist, Trajectory
from rc_car.scripts.py_AStar import CSpace
plt.ion()

class RRTSTAR(object):
    def __init__(self, env_map, max_step_size = 0.5, max_itr=5000, stop_on_goal=False, p_bias=0.05, k=5, show_animation=False):
        self.max_step_size = max_step_size
        self.max_itr = max_itr
        self.stop_on_goal = stop_on_goal
        self.tree = Tree()
        self.k = k
        self.map = env_map
        self.env_rows, self.env_cols = env_map.shape
        self.env_yaw_range = 2*np.pi
        self.p_bias = p_bias
        self.show_animation = show_animation
        

    def find_path(self, start, goal):
        print(self.env_rows)
        print(self.env_cols)
        itr = 0
        self.tree.AddVertex(start)
        path_found = False
        path_cost = np.inf
        path_idx = 1
        while itr < self.max_itr:
            
            # sample
            x_random = self.sample(goal)

            # find nearest neighbor
            x_near_idx, x_near = self.tree.GetNearestVertex(x_random)
            
            # extend
            x_new = self.extend(x_random, x_near)
            if path_found and np.array_equal(x_new, goal):
                continue
            # add vertex and edge
            if not self.is_in_collision(x_new) and self.local_planner(x_near, x_new, calc_configs_dist(x_near, x_new)):
                x_new_idx = self.tree.AddVertex(x_new)
                self.tree.AddEdge(x_near_idx, x_new_idx)

                # find k neaerst neihbors
                self.k = int(np.log(len(self.tree.vertices)))
                knn_ids = self.tree.GetKNN(x_new, self.k)

                # check for collisions
                collisions = []
                for knn_id in knn_ids:
                    if self.local_planner(x_new, self.tree.vertices[knn_id].conf, calc_configs_dist(x_new, self.tree.vertices[knn_id].conf)):
                        collisions.append(True)
                    else:
                        collisions.append(False)

                # check for better parrent
                for idx, knn_id in enumerate(knn_ids):
                    if collisions[idx] and \
                    self.tree.vertices[knn_id].cost + calc_configs_dist(x_new, self.tree.vertices[knn_id].conf) < self.tree.vertices[x_new_idx].cost:
                        self.tree.AddEdge(knn_id, x_new_idx)

                # check for better child
                for idx, knn_id in enumerate(knn_ids):
                    if collisions[idx] and \
                    self.tree.vertices[x_new_idx].cost + calc_configs_dist(x_new, self.tree.vertices[knn_id].conf) < self.tree.vertices[knn_id].cost:
                        self.tree.AddEdge(x_new_idx, knn_id)
                
                if self.show_animation:
                    self.draw_graph(x_new, start, goal)
           
                if np.array_equal(x_new, goal):
                    path_found = True
                    self.p_bias = 0.0
                    path, path_cost = self.get_shortest_path(goal)

            if path_found and self.stop_on_goal:
                return path , path_cost
            if path_found and not self.stop_on_goal:
                path, current_cost = self.get_shortest_path(goal)
                if current_cost < path_cost:
                    path_cost = current_cost
                    np.save('path'+str(path_idx), np.array(path))
                    path_idx += 1
                    print(f'itr: {itr} path_cost: {path_cost}')
            itr += 1
            if itr%1000 ==0:
                print(f'itr: {itr}')
        if path_found:
            return self.get_shortest_path(goal)
        return None
    
    def sample(self, goal):
        if np.random.rand() < self.p_bias:
            return goal
        return np.array([np.random.randint(self.env_cols), np.random.randint(self.env_rows), 0], dtype=int)#np.random.rand() * self.env_yaw_range])
    
    def extend(self, x_random, x_near):
        dist = np.linalg.norm(x_random - x_near, ord=2)
        if dist > self.max_step_size:
            x_random_new = np.array(x_near + self.max_step_size * (x_random - x_near) / dist, dtype=int)
            x_random_new[0] = min(x_random_new[0], self.env_cols-1)
            x_random_new[0] = max(x_random_new[0], 0)
            x_random_new[1] = min(x_random_new[1], self.env_rows-1)
            x_random_new[1] = max(x_random_new[1], 0)
            return np.array(x_random_new, dtype=int)
        return x_random
    
    def is_in_collision(self, x_new):
        if self.map[x_new[1]][x_new[0]] == 0:
            return False
        return True
    
    def local_planner(self, config1, config2, dist):
        configs = np.linspace(config1, config2, num=max(3,int(dist)) ,dtype=int)
        for config in configs:
            if self.is_in_collision(config):
                return False
        return True
    
    def get_shortest_path(self, goal):
        '''
        Returns the path and cost from some vertex to Tree's root
        @param dest - the id of some vertex
        return the shortest path and the cost
        '''
        path = []
        goal_idx = self.tree.getIndexForState(goal)
        path.append(goal_idx)
        while path[-1] != 0:
            path.append(self.tree.edges[path[-1]])
        path = np.array([self.tree.vertices[idx].conf for idx in path][::-1])
        return path , self.compute_cost(path)
    
    def compute_cost(self, path):
        '''
        Compute the total cost of the given path
        @param path a numpy array with an ordered set of configurations
        return the total cost
        '''
        path_cost = 0.0
        for i in range(len(path)-1):
            path_cost += calc_configs_dist(path[i], path[i+1])
        return path_cost
    
    def draw_graph(self,x_new, start, goal):
        robot_radius = 0.5
        # plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.scatter(start[0], start[1], s=100, c='g')
        plt.scatter(goal[0],goal[1], s=100, c='r')
        plt.scatter(x_new[0], x_new[1], s=10, c='b')
        plt.xlim([0, self.env_cols])
        plt.ylim([0, self.env_rows])
        plt.imshow(self.map, origin="lower")
        plt.pause(0.01)
    
    def draw_path(self, path, trajectory:Trajectory):
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        for i in range(1, len(path)):
            plt.plot([path[i-1][0], path[i][0]], [path[i-1][1], path[i][1]], c='m')
        plt.plot(trajectory.cx, trajectory.cy, "-r", label="course")
        plt.xlim([0, self.env_cols])
        plt.ylim([0, self.env_rows])
        plt.imshow(self.map, origin="lower")
        plt.pause(5)
    
    def draw_paths(self, path_num):
        for path_id in range(1, path_num+1):
            path = np.load('path'+str(path_id)+'.npy')
            plt.cla()
            plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
            for i in range(1, len(path)):
                plt.plot([path[i-1][0], path[i][0]], [path[i-1][1], path[i][1]], c='m')
            plt.xlim([0, self.env_cols])
            plt.ylim([0, self.env_rows])
            plt.imshow(self.map, origin="lower")
            plt.pause(1)



def inflate(map_, inflation):#, resolution, distance):
    cells_as_obstacle = int(inflation) #int(distance/resolution)
    original_map = map_.copy()
    inflated_map = map_.copy()
    rows, cols = inflated_map.shape
    for j in range(cols):
        for i in range(rows):
            if original_map[i,j] != 0:
                i_min = max(0, i-cells_as_obstacle)
                i_max = min(rows, i+cells_as_obstacle)
                j_min = max(0, j-cells_as_obstacle)
                j_max = min(cols, j+cells_as_obstacle)
                inflated_map[i_min:i_max, j_min:j_max] = 100
    return inflated_map       

    



def main():
    map_original = np.array(np.load('maze_test.npy'), dtype=int)
    env_rows, env_cols = map_original.shape
    resolution=0.05000000074505806
    origin_x=-4.73 
    origin_y=-5.66
    converter = CSpace(resolution, origin_x, origin_y,env_rows, env_cols )

    # map_ = np.array(np.load('maze_1.npy'), dtype=int)
    map_original = np.array(np.load('maze_test.npy'), dtype=int)
    map_ = inflate(map_original, 0.4/resolution)
    start=converter.meter2pixel([0.0,0.0])
    goal = converter.meter2pixel([6.22, -4.22])
    start = np.array([start[0], start[1], 0.0], dtype= int)
    goal = np.array([goal[0], goal[1], 0], dtype=int)
    print(start)
    print(goal)
    rrt_planner = RRTSTAR(env_map=map_, max_step_size=20, max_itr=15000, stop_on_goal=True, p_bias=0.05, show_animation=False)
    # rrt_planner.draw_paths(22)
    # path = np.load('path1.npy')
    path, cost = rrt_planner.find_path(start, goal)
    trajectory = Trajectory(dl=0.5, path = path, TARGET_SPEED=1.0)
    
    rrt_planner.draw_path(path, trajectory)
    print(path)


if __name__ == "__main__":
    main()


