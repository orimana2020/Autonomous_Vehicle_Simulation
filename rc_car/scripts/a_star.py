import numpy as np
import matplotlib.pyplot as plt
import heapq


class A_Star(object):
    def __init__(self, map: np.array):
        self.map = map
        self.rows, self.cols = map.shape
        self.inflated_map = self.inflate()

    def inflate(self):#, resolution, distance):
        cells_as_obstacle = 1 #int(distance/resolution)
        inflated_map = self.map.copy()
        for i in range(self.cols):
            for j in range(self.rows):
                if self.map[j][i] == 1:
                    j_min = max(0, j-cells_as_obstacle)
                    j_max = min(self.rows, j+cells_as_obstacle)
                    i_min = max(0, i-cells_as_obstacle)
                    i_max = min(self.cols, i+cells_as_obstacle)
                    for j in range(j_min,j_max):
                        for i in range(i_min, i_max):
                            inflated_map[j][i] = 1
        return inflated_map
    

# using heapify to convert list into heap
# heapq.heapify(li)
 
# printing created heap
# print("The created heap is : ", end="")
# print(list(li))
 
# using heappush() to push elements into heap
# pushes 4
# heapq.heappush(li, 4)
 
# printing modified heap
# print("The modified heap after push is : ", end="")
# print(list(li))
 
# using heappop() to pop smallest element
# print("The popped and smallest element is : ", end="")
# print(heapq.heappop(li))       
        

    
    def find_path(self, start, goal):
        open_list = [(0, start)]
        heapq.heapify(open_list)
        while open_list:
            current_vertex = heapq.heappop(open_list)



map_ = np.array([[0,0,0,1,1,0,1],
                 [0,0,0,1,0,0,0],
                 [0,1,0,0,0,1,0],
                 [0,0,1,0,0,0,0],
                 [0,0,0,1,0,0,0],
                 [0,0,0,0,0,0,0],
                 [0,0,0,0,0,0,0],
                 [0,0,1,0,0,0,0],
                 [0,0,0,1,0,0,0],
                 [0,0,1,0,0,0,0],
                 [0,0,0,0,0,0,1],
                 [0,0,0,0,0,0,1],
                 [0,0,0,0,0,0,1],])

astar = A_Star(map_)
fig = plt.figure()
ax = fig.add_subplot()
for i in range(astar.cols):
    for j in range(astar.rows):
        # if map_[j][i] == 1:
        if astar.inflated_map[j][i] == 1:
            ax.scatter(i,j, c='red')
        else:
            ax.scatter(i,j,c='grey')
plt.show()

