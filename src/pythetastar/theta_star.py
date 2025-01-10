import numpy as np
import time
import math

# Utility class to handle nodes more easily
class Node:
    OBSTACLE = 1
    FREE = 0

    def __init__(self, x, y):
        self.pos = [x, y]
        self.reset()

    def reset(self):
        self.parent: Node = None
        self.local = None
        self.h_score = 0.0
        self.g_score = math.inf

    def __repr__(self):
        return f'<Node: {self.pos.__repr__()}>'

class NoPathFoundException(BaseException):
    pass

def dist(p1: Node, p2: Node):
    if p1 is None or p2 is None:
        return math.inf
    
    sqr = (p1.pos[0] - p2.pos[0])**2 + (p1.pos[1]-p2.pos[1])**2
    return math.sqrt(sqr)


def update_grid_blocked_cells(blocked_cells, grid):
    for x, y in blocked_cells:
        grid[x, y] = Node.OBSTACLE if grid[x, y] == Node.FREE else Node.FREE


def line_of_sight_neighbours(a, b, obs):
    (xa, ya), (xb, yb) = a, b
    dx, dy = xb - xa, yb - ya

    try:
        if dx != 0 and dy != 0:
            # Diagonal case
            return obs[xa - 1 * (dx < 0), ya - 1 * (dy < 0)] == Node.FREE
        elif dx == 0:
            # Vertical case
            yobs = ya - 1 * (dy < 0)
            return obs[xa, yobs] == Node.FREE or obs[xa - 1, yobs] == Node.FREE
        else:
            # Horizontal case
            xobs = xa - 1 * (dx < 0)
            return obs[xobs, ya] == Node.FREE or obs[xobs, ya - 1] == Node.FREE
    except IndexError:
        return True


def line_of_sight(a, b, grid):
    '''
    Bresenham (1950) fast line of sight algorithm for computer graphics
    '''
    (x0, y0), (x1, y1) = a.pos, b.pos
    dx, dy = x1 - x0, y1 - y0
    f = 0
    
    if dy < 0:
        dy = -dy
        sy = -1
    else:
        sy = 1
    
    if dx < 0:
        dx = -dx
        sx = -1
    else:
        sx = 1
    
    if dx >= dy:
        while x0 != x1:
            f = f + dy
            if f >= dx:
                if grid[x0 + int((sx-1)/2), y0 + int((sy-1)/2)] == Node.OBSTACLE:
                    return False
                y0 = y0 + sy
                f = f - dx
            if f != 0 and grid[x0 + int((sx-1)/2), y0 + int((sy-1)/2)] == Node.OBSTACLE:
                return False
            if dy == 0 and grid[x0 + int((sx-1)/2), y0] == Node.OBSTACLE and grid[x0 + int((sx-1)/2), y0-1] == Node.OBSTACLE:
                return False
            x0 = x0 + sx
    else:
        while y0 != y1:
            f = f + dx
            if f >= dy:
                if grid[x0 + int((sx-1)/2), y0 + int((sy-1)/2)] == Node.OBSTACLE:
                    return False
                x0 = x0 + sx
                f = f - dy
            if f != 0 and grid[x0 + int((sx-1)/2), y0 + int((sy-1)/2)] == Node.OBSTACLE:
                return False
            if dx == 0 and grid[x0, y0 + int((sy-1)/2)] == Node.OBSTACLE and grid[x0-1, y0 + int((sy-1)/2)] == Node.OBSTACLE:
                return False
            y0 = y0 + sy
    return True


def path_length(path):
    length = 0
    prev = None
    for pt in path:
        if prev is not None:
            length += dist(prev, pt)
        prev = pt
    return length


# Return all the children (neighbors) of a specific node
def children(node: Node, grid, obs, allow_diagonals=False, check_line_of_sight=True) -> list[Node]:
    pos = np.array(node.pos)
    children = []
        
    if allow_diagonals:
        directions = np.array([[1,0],[0,1],[1,1],[-1,-1],[1,-1],[-1,1],[0,-1],[-1,0]])
    else:
        directions = np.array([[1,0],[0,1],[0,-1],[-1,0]])
    for d in pos + directions:
        if 0 <= d[0] < grid.shape[0] and 0 <= d[1] < grid.shape[1] and (not check_line_of_sight or line_of_sight_neighbours(node.pos, grid[d[0], d[1]].pos, obs)):
            children.append(grid[d[0], d[1]])
    return children
    

def update_vertex(current: Node, node: Node, grid, obs):
    if current.parent and line_of_sight(current.parent, node, obs):
        # Path 2
        # If in line of sight, we connect to the parent, it avoid unecessary grid turns
        new_g = current.parent.g_score + dist(current.parent, node)
        if new_g < node.g_score:
            node.g_score = new_g
            node.parent = current.parent
            node.local = current
    else:
        # Path 1
        new_g = current.g_score + dist(current, node)
        if new_g < node.g_score:
            node.g_score = new_g
            node.parent = current
            node.local = current

# Return the path computed by the A* optimized algorithm from the start and goal points
def find_path(start: Node, goal: Node, grid, obs, h_cost_weight, openset: set[Node] = set(), closedset: set[Node] = set()):
    if len(openset) == 0:
        openset.add(start)

    i = 0
    while openset and min(map(lambda o: o.g_score + h_cost_weight * o.h_score, openset)) < goal.g_score + h_cost_weight * goal.h_score:
        i = i + 1
        current = min(openset, key=lambda o: o.g_score + h_cost_weight * o.h_score)

        openset.remove(current)
        closedset.add(current)

        # Loop through the node's children/siblings
        for node in children(current, grid, obs, allow_diagonals=True):
            # If it is already in the closed set, skip it
            if node in closedset:
                continue

            if node not in openset:
                node.g_score = float('inf')
                node.h_score = dist(node, goal)
                node.parent = None
                openset.add(node)

            update_vertex(current, node, grid, obs)
            

    if not goal.parent:
        raise NoPathFoundException()
    
    path = []
    current = goal
    while current.parent:
        path.append(current)
        current = current.parent
    path.append(current)
    return path[::-1]


def path_blocked(grid_obs, path):
    prev = None
    for node in path:
        if prev is not None and not line_of_sight(prev, node, grid_obs):
            return True
        prev = node
    return False

def theta_star(
        start: tuple[float, float], 
        goal: tuple[float, float], 
        grid_obs: np.array, 
        h_cost_weight: float = 1.7, 
        new_blocked_cells=[],
        allow_replanning = True
        ):
    durations = []
    lengths = []
    paths = []

    x, y = np.mgrid[0:grid_obs.shape[0]+1, 0:grid_obs.shape[1]+1]
    grid = np.vectorize(Node)(x, y)
    start = grid[start]
    goal = grid[goal]
    goal.h_score = 0 
    start.g_score = 0
    start.h_score = dist(start, goal)

    new_blocked_cells = iter(new_blocked_cells)
    openset = set()
    closedset = set()

    t1 = time.time()
    duration = 0

    i = 0
    while True:
        i += 1
        path = find_path(start, goal, grid, grid_obs, h_cost_weight, openset, closedset)
        
        duration = abs(time.time() - t1)
        durations.append(duration)
        lengths.append(path_length(path))
        paths.append(list(map(lambda n: n.pos, path)))

        if not allow_replanning:
            break
        else:
            try:
                blocked_cells = next(new_blocked_cells)
                update_grid_blocked_cells(blocked_cells, grid_obs)
            except StopIteration:
                break

        t1 = time.time()

        if path_blocked(grid_obs, path):
            openset = set()
            closedset = set()
            goal.reset()
        
    explored_nodes = openset.union(closedset)
    final_path = list(map(lambda n: n.pos, path))
    return final_path, explored_nodes, durations, lengths, paths