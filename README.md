# PyThetaStar
Python implementation of the Theta* algorithm (Daniel, Nash, "Theta star, Any-angle path planning on grids"). This is a modified and simplified single-file adaptation of the implementation [found here.](https://github.com/rhidra/phi_star_2d/blob/master/theta_star.py)

Example of path generated:

![Path generated](https://github.com/alek5k/pythetastar/raw/master/examples/path.png)

Dynamic replanning is allowed by modifying `new_blocked_cells` which is an input to the `theta_star` function.

# Installation
`pip install pythetastar`

# Basic Usage
```python
import numpy as np
from pythetastar import theta_star

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
print(result_path) 
# output: [[0, 0], [1, 2], [5, 4], [6, 4], [8, 8]]
```

Plotting:
```python
import matplotlib.pyplot as plt

xs = [n[0] for n in result_path]
ys = [n[1] for n in result_path]

plt.imshow(grid.T, origin='lower', extent=[0, grid_width, 0, grid_height], cmap='cividis')
# Transpose aligns rows (axis 0) with y-axis and columns (axis 1) with x-axis.
plt.grid(color='black', linestyle='-', linewidth=0.5)
plt.xticks(range(grid_width))  
plt.yticks(range(grid_height))
plt.plot(xs, ys, 'r-')
plt.scatter(xs, ys)
plt.show()
```