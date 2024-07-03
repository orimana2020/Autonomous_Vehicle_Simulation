import numpy as np

path = np.load('path2_meter.npy')
loops = 3
looped_path = []
for _ in range(loops):
    for coords in path:
        looped_path.append(coords)
np.save('path3_meter', np.array(looped_path))
print(len(path))
print(len(looped_path))
