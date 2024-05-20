import numpy as np
import heapq
import matplotlib.pyplot as plt
from utils import Ackermann_ARC
sec1 = np.load('path_race4_section1.npy')
sec1 = sec1[:-3]
sec2 = np.load('path_race4_section2.npy')
sec2 = sec2[:-3]
sec3 = np.load('path_race4_section3.npy')
sec3 = sec3[:-3]
print(len(sec1))
race4_full_path_meter = []
for x,y,_ in sec1:
    race4_full_path_meter.append([x,y])
for x,y,_ in sec2:
    race4_full_path_meter.append([x,y])
for x,y,_ in sec3:
    race4_full_path_meter.append([x,y])
np.save('race4_full_path_meter',race4_full_path_meter)
race4_full_path_meter = np.load('race4_full_path_meter.npy')
plt.scatter(race4_full_path_meter[:,0],race4_full_path_meter[:,1])
plt.show()