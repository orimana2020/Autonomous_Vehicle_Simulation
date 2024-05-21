import numpy as np
import matplotlib.pyplot as plt
from py_Utils import Tree, Ackermann_ARC, calc_configs_dist
from py_AStar import CSpace
plt.ion()

class KINORRTSTAR(object):
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
        self.ackerman_kino = Ackermann_ARC(wheelbase=0.34)
        resolution=0.05000000074505806
        origin_x=-4.73 
        origin_y=-5.66
        self.converter = CSpace(resolution, origin_x, origin_y, self.env_rows, self.env_cols)
        

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
            # x_random = self.converter.meter2pixel([1,0])

            # find nearest neighbor
            x_near_idx, x_near = self.tree.GetNearestVertex(x_random)
            
            # extend
            x_new = self.extend(x_random, x_near)
            
            if path_found and np.array_equal(x_new, goal):
                continue
            # add vertex and edge
            if not self.is_in_collision(x_new) and self.local_planner(x_near, x_new):
                x_new_idx = self.tree.AddVertex(x_new)
                self.tree.AddEdge(x_near_idx, x_new_idx, self.arc_length)
                self.tree.vertices[x_new_idx].set_waypoints(self.waypoints)
                self.tree.vertices[x_new_idx].set_yaw(self.yaw)

                # find k neaerst neihbors
                self.k = int(np.log(len(self.tree.vertices)))
                knn_ids = self.tree.GetKNN(x_new, self.k)

                # check for better parrent
                for knn_id in knn_ids:
                    if self.local_planner(self.tree.vertices[knn_id].conf, x_new) and \
                    self.tree.vertices[knn_id].cost + self.arc_length < self.tree.vertices[x_new_idx].cost:
                        self.tree.AddEdge(knn_id, x_new_idx, self.arc_length)
                        self.tree.vertices[x_new_idx].set_waypoints(self.waypoints)
                        self.tree.vertices[x_new_idx].set_yaw(self.yaw)

                # check for better child
                for knn_id in knn_ids:
                    if self.local_planner(x_new, self.tree.vertices[knn_id].conf) and \
                    self.tree.vertices[x_new_idx].cost + self.arc_length < self.tree.vertices[knn_id].cost:
                        self.tree.AddEdge(x_new_idx, knn_id, self.arc_length)
                        self.tree.vertices[knn_id].set_waypoints(self.waypoints)
                        self.tree.vertices[knn_id].set_yaw(self.yaw)
                
                if self.show_animation:
                    self.draw_graph(x_new, start, goal)
           
                if np.array_equal(x_new, goal):
                    print(x_new)
                    print(goal)
                    path_found = True
                    self.p_bias = 0.0
                    self.path, self.path_idx ,self.path_cost = self.get_shortest_path(goal)

            if path_found and self.stop_on_goal:
                return self.path, self.path_idx ,self.path_cost
            if path_found and not self.stop_on_goal:
                self.path, self.path_idx ,self.current_cost = self.get_shortest_path(goal)
                if self.current_cost < self.path_cost:
                    self.path_cost = self.current_cost
                    np.save('path'+str(path_idx), np.array(self.path))
                    path_idx += 1
                    print(f'itr: {itr} path_cost: {path_cost}')
            itr += 1
            if itr%1000 ==0:
                print(f'itr: {itr}')
        if path_found:
            return self.get_shortest_path(goal)
        self.draw_tree()
        return None
    
    def sample(self, goal):
        if np.random.rand() < self.p_bias:
            return goal
        return np.array([np.random.randint(self.env_cols), np.random.randint(self.env_rows), np.pi*2*np.random.rand()])#np.random.rand() * self.env_yaw_range])
    
    def extend(self, x_random, x_near):
        dist = np.linalg.norm(x_random - x_near, ord=2)
        if dist > self.max_step_size:
            x_random_new = np.array(x_near + self.max_step_size * (x_random - x_near) / dist, dtype=int)
            x_random_new[0] = min(x_random_new[0], self.env_cols-1)
            x_random_new[0] = max(x_random_new[0], 0)
            x_random_new[1] = min(x_random_new[1], self.env_rows-1)
            x_random_new[1] = max(x_random_new[1], 0)
            return np.array(x_random_new)
        return x_random
    
    def is_in_collision(self, x_new):
        if self.map[int(x_new[1])][int(x_new[0])] == 0:
            return False
        return True
    
    def local_planner(self, x_near, x_new):
        if x_near[0] == x_new[0] or x_near[1] == x_new[1]:
            return False
        x_near_meter = self.converter.pixel2meter(x_near)
        x_new_meter = self.converter.pixel2meter(x_new)
        self.waypoints_meter, self.total_time, self.arc_length, self.yaw = self.ackerman_kino.get_arc(x_near_meter, x_new_meter, velocity=1.0)
        self.waypoints = self.converter.pathmeter2pathindex(self.waypoints_meter)
       
        for config in self.waypoints:
            if self.is_in_collision(config):
                return False
        return True
    
    def get_shortest_path(self, goal):
        '''
        Returns the path and cost from some vertex to Tree's root
        @param dest - the id of some vertex
        return the shortest path and the cost
        '''
        path_idx = []
        goal_idx = self.tree.getIndexForState(goal)
        path_idx.append(goal_idx)
        while path_idx[-1] != 0:
            path_idx.append(self.tree.edges[path_idx[-1]])
        path = np.array([self.tree.vertices[idx].conf for idx in path_idx][::-1])
        return path, path_idx[::-1] , self.compute_cost(path)
    
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
    
    def draw_path(self, path, path_idx):
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        for idx in path_idx[1:]:
            arc_waypoints = self.tree.vertices[idx].waypoints
            plt.scatter(arc_waypoints[:,0], arc_waypoints[:,1], c='r',s=10)
        for i in range(len(path)):
            plt.scatter( path[i][0], path[i][1], c='y')    
        # for i in range(1, len(path)):
        #     plt.scatter([path[i-1][0], path[i][0]], [path[i-1][1], path[i][1]], c='y')
        plt.xlim([0, self.env_cols])
        plt.ylim([0, self.env_rows])
        plt.imshow(self.map, origin="lower")
        plt.pause(50)
    
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

    def draw_tree(self):
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        for i in range(len(self.tree.vertices)):
            conf = self.tree.vertices[i].conf
            plt.scatter(conf[0], conf[1])
        plt.xlim([0, self.env_cols])
        plt.ylim([0, self.env_rows])
        plt.imshow(self.map, origin="lower")
        plt.pause(30)




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



        




        



def main():
    map_original = np.array(np.load('maze_test.npy'), dtype=int)
    env_rows, env_cols = map_original.shape
    resolution=0.05000000074505806
    origin_x=-4.73 
    origin_y=-5.66
    converter = CSpace(resolution, origin_x, origin_y,env_rows, env_cols )

    # map_ = np.array(np.load('maze_1.npy'), dtype=int)
    
    map_ = inflate(map_original, 0.2/resolution)
    start=converter.meter2pixel([0.0,0.0])
    goal = converter.meter2pixel([6.22, -4.22])
    start = np.array([start[0], start[1], 0.0])
    goal = np.array([goal[0], goal[1], 0.0])
    print(start)
    print(goal)
    rrt_planner = KINORRTSTAR(env_map=map_, max_step_size=20, max_itr=30000, stop_on_goal=False, p_bias=0.05, show_animation=False)
    # rrt_planner.draw_paths(22)
    path, path_idx,cost = rrt_planner.find_path(start, goal)
    rrt_planner.draw_path(path, path_idx)


if __name__ == "__main__":
    main()


