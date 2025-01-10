import numpy as np
from pythetastar import theta_star
import matplotlib.pyplot as plt

grid = np.array([
    [0, 0, 0, 0, 1, 0, 1, 1],
    [1, 1, 0, 1, 1, 0, 1, 1],
    [0, 1, 0, 0, 1, 0, 0, 1],
    [1, 1, 1, 0, 1, 1, 0, 1],
    [1, 1, 1, 0, 1, 0, 1, 1],
    [0, 0, 1, 0, 1, 0, 1, 1],
    [0, 1, 1, 0, 0, 0, 1, 1],
    [0, 1, 0, 1, 1, 0, 0, 0]
])
grid_width, grid_height = grid.shape
start = (0, 0)
goal = (grid_width, grid_height)

result_path, node_set, durations, lengths, paths = theta_star(start, goal, grid)
print(result_path) # output: [[0, 0], [1, 2], [5, 4], [6, 4], [8, 8]]

xs = [n[0] for n in result_path]
ys = [n[1] for n in result_path]

plt.imshow(grid.T, origin='lower', extent=[0, grid_width, 0, grid_height], cmap='cividis') # Transpose aligns rows (axis 0) with y-axis and columns (axis 1) with x-axis.
plt.grid(color='black', linestyle='-', linewidth=0.5)
plt.xticks(range(grid_width))  
plt.yticks(range(grid_height))
plt.plot(xs, ys, 'r-')
plt.scatter(xs, ys)
plt.show()