from enum import Enum
import copy
from config import BD_LOC, LARGE_BUILD

grid_size = 50  # Define the size of your grid
grid2 = [[[0 for _ in range(grid_size)] for _ in range(grid_size)] for _ in range(grid_size)]
blocks_no_longer_walkable = []

def initialize_grid_with_structures(grid_size):
    # Initialize an empty grid with all cells as obstacles
    grid = [[[1 for _ in range(grid_size)] for _ in range(grid_size)] for _ in range(grid_size)]

    # Make the bottom layer (z=0) walkable
    for x in range(grid_size):
        for y in range(grid_size):
            grid[x][0][y] = 0  # Set the bottom layer cells to walkable (0)

    return grid

def update_grid_with_structure(grid, structure):
    global blocks_no_longer_walkable
    # for structure in structures:        
    x, z, y = structure  # Ensure the order matches your design

    if 0 <= x < grid_size and 0 <= y < grid_size and 0 <= z < grid_size:
        grid[x][z][y] = 0  # Mark as walkable
        grid[x][z-1][y] = 1  # Mark the cell below the structure as not walkable
        blocks_no_longer_walkable.append((x, z-1, y))
    
    return grid

def bfs_vertical_path(grid, path_start, path_end):
    global blocks_no_longer_walkable
    # for structure in structures:        
    x, z, y = path_end  # Ensure the order matches your design

    # add all the blocks directly under path_end in blocks_no_longer_walkable to the grid as walkable
    prev_grid = copy.deepcopy(grid)

    if 0 <= x < grid_size and 0 <= y < grid_size and 0 <= z < grid_size:
        for i in range(z):
            grid[x][z-i][y] = 0  # Mark as walkable

    # recalculate bfs with the new grid and looking vertically
    path_coords, num_steps = bfs_3d_vertical(grid, path_start, path_end)

    # reset the grid to the original grid
    grid = prev_grid
    if 0 <= x < grid_size and 0 <= y < grid_size and 0 <= z < grid_size:
        grid[x][z][y] = 0  # Mark as walkable
        grid[x][z-1][y] = 1  # Mark the cell below the structure as not walkable
        blocks_no_longer_walkable.append((x, z-1, y))
    
    return grid, path_coords, num_steps

def update_grid_with_structure_not_walkable(grid, structure):
    # for structure in structures:        
    x, z, y = structure  # Ensure the order matches your design

    if 0 <= x < grid_size and 0 <= y < grid_size and 0 <= z < grid_size:
        grid[x][z-1][y] = 1  # Mark the cell below the structure as walkable
    
    return grid 

class Node:
    def __init__(self, row, col, depth, is_obs, h=0):
        self.row = row
        self.col = col
        self.depth = depth
        self.is_obs = is_obs
        self.cost = None
        self.parent = None

def is_valid_position_3d(grid, position):
    rows, cols, depths = len(grid), len(grid[0]), len(grid[0][0])
    row, col, depth = position
    return 0 <= row < rows and 0 <= col < cols and 0 <= depth < depths

def is_goal_reached_3d(current_node, goal_node):
    return (current_node.row == goal_node.row and 
            current_node.col == goal_node.col and 
            current_node.depth == goal_node.depth)

def is_valid_start_goal_3d(grid, start, goal):
    return (is_valid_position_3d(grid, start) and 
            is_valid_position_3d(grid, goal) and 
            not grid[start[0]][start[1]][start[2]] and 
            not grid[goal[0]][goal[1]][goal[2]])

def rework_path_3d(current_node, holding_block):
    path = []
    while current_node:
        path.append(([current_node.row, current_node.col, current_node.depth], holding_block))
        current_node = current_node.parent
    return path[::-1]

def start_search_3d(grid, start, goal):
    rows, cols, depths = len(grid), len(grid[0]), len(grid[0][0])
    start_node = Node(start[0], start[1], start[2], False)
    goal_node = Node(goal[0], goal[1], goal[2], False)
    visited = [[[False for _ in range(depths)] for _ in range(cols)] for _ in range(rows)]
    return start_node, goal_node, visited, rows, cols, depths

# Calculates BFS search to find path from start_coords to end_coord on grid
# Returns a list of coordinates of each block of the path
def bfs_3d(grid, start, goal):
    # if it goal is the Block Depot then it is not holding a block
    holding_block = True
    if goal == BD_LOC:
        holding_block = False 

    if not is_valid_start_goal_3d(grid, start, goal):
        print("Invalid start or goal")
        return [], 0

    start_node, goal_node, visited, rows, cols, depths = start_search_3d(grid, start, goal)
    queue = [start_node]
    visited[start_node.row][start_node.col][start_node.depth] = True

    steps = 0
    while queue:
        current_node = queue.pop(0)
        steps += 1

        if is_goal_reached_3d(current_node, goal_node):
            return rework_path_3d(current_node, holding_block), steps

        if LARGE_BUILD:
            for dr, dc, dd in [(0, 1, 0), (1, 0, 0), (0, -1, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1),
                        (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),  
                        (1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1),  
                        (1, 0, -1), (0, 1, -1), (-1, 0, -1), (0, -1, -1),
                        (1, 1, 1), (1, -1, 1), (-1, 1, 1), (-1, -1, 1), 
                        (1, 1, -1), (1, -1, -1), (-1, 1, -1), (-1, -1, -1)]:
                r, c, d = current_node.row + dr, current_node.col + dc, current_node.depth + dd
                if is_valid_position_3d(grid, (r, c, d)) and not grid[r][c][d] and not visited[r][c][d]:
                    visited[r][c][d] = True
                    neighbor = Node(r, c, d, False)
                    neighbor.parent = current_node
                    queue.append(neighbor)

        else:
        # This is no diagonal movement
            for dr, dc, dd in [ (1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1),   # left right forward back
                                (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0), # change in x and z but no change in y 
                                (1, 2, 0), (1, -2, 0), (-1, 2, 0), (-1, -2, 0), # 2 block step down 
                                (0, 2, -1), (0, -2, -1), (0, -2, 1), (0, 2, 1), #2 block step down 
                                (0, 1, -1), (0, -1, -1), (0, -1, 1), (0, 1, 1),  # change in x and z but no change in y 
                            ]: # (0, 1, 0), (0, -1, 0) up and down 


                r, c, d = current_node.row + dr, current_node.col + dc, current_node.depth + dd
                if is_valid_position_3d(grid, (r, c, d)) and not grid[r][c][d] and not visited[r][c][d]:
                    visited[r][c][d] = True
                    neighbor = Node(r, c, d, False)
                    neighbor.parent = current_node
                    queue.append(neighbor)

    # return a negative number if no path found
    print("No path found using BFS in 3D")
    return [], -1

# Calculates BFS search to find path from start_coords to end_coord on grid
# Returns a list of coordinates of each block of the path
# this BFS will look for vertical paths
def bfs_3d_vertical(grid, start, goal):

    # if it goal is the Block Depot then it is not holding a block
    holding_block = True
    if goal == BD_LOC:
        holding_block = False


    if not is_valid_start_goal_3d(grid, start, goal):
        print("Invalid start or goal")
        return [], 0

    start_node, goal_node, visited, rows, cols, depths = start_search_3d(grid, start, goal)
    queue = [start_node]
    visited[start_node.row][start_node.col][start_node.depth] = True

    steps = 0
    while queue:
        current_node = queue.pop(0)
        steps += 1

        if is_goal_reached_3d(current_node, goal_node):
            return rework_path_3d(current_node, holding_block), steps

        # This is no diagonal movement
        for dr, dc, dd in [ (1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1), (0, 1, 0), (0, -1, 0),   # left right forward back up down
                            (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0), # change in x and z but no change in y 
                            (1, 2, 0), (1, -2, 0), (-1, 2, 0), (-1, -2, 0), # 2 block step down 
                            (0, 1, -1), (0, -1, -1), (0, -1, 1), (0, 1, 1),  # change in x and z but no change in y 
                            (0, 2, -1), (0, -2, -1), (0, -2, 1), (0, 2, 1) #2 block step down 
                         ]: # (0, 1, 0), (0, -1, 0) up and down 

        # 26 connected edges search     
        # for dr, dc, dd in [(0, 1, 0), (1, 0, 0), (0, -1, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1),  # Orthogonal directions
        #                 (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),  # Edge-diagonal on same level
        #                 (1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1),  # Edge-diagonal with one level difference
        #                 (1, 0, -1), (0, 1, -1), (-1, 0, -1), (0, -1, -1),
        #                 (1, 1, 1), (1, -1, 1), (-1, 1, 1), (-1, -1, 1),  # Corner-diagonal directions
        #                 (1, 1, -1), (1, -1, -1), (-1, 1, -1), (-1, -1, -1)]:
            
            r, c, d = current_node.row + dr, current_node.col + dc, current_node.depth + dd
            if is_valid_position_3d(grid, (r, c, d)) and not grid[r][c][d] and not visited[r][c][d]:
                visited[r][c][d] = True
                neighbor = Node(r, c, d, False)
                neighbor.parent = current_node
                queue.append(neighbor)

    # return a negative number if no path found
    print("No path found using BFS in 3D")
    return [], -1