import numpy as np
import heapq

from utils import Ackermann_ARC

a = np.array([1,1,1])
b= np.array([1,1,3])
print((a[:2]==b[:2]).all())