import copy
from config import BD_LOC
from map_data import *

def breadth_first_search(grid, start, goal, holding_block, prioritize_vertical=False):
    """
    Perform modified BFS in a 3D grid.
    
    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
        start (tuple): A tuple containing the (x, z, y) coordinate of the starting cell in a path.
                       The initial starting position can be configurable in config.py
        goal (tuple): A tuple containing the (x, z, y) coordinate of the ending cell in a path.
                      This typically is either the block depot or a block coordinate in the blueprint.
        #TODO: clarify if iw_id is int or not
        inchworm_id (int): An ID that identifies which inchworm grid, start, and goal is being taken in.
        prioritize_vertical (boolean): A flag that determines if vertical neighbors are prioritized.
    Returns:
        
    """
    print(f"BFS called with start: {start}, goal: {goal}, prioritize_vertical: {prioritize_vertical}")
    
    neighbor_directions = set_neighbors(prioritize_vertical)
    
    if is_valid_start_goal_3d(grid, *start, *goal):
        #TODO: put function in map_data
        start_cell = Cell(start[0], start[1], start[2])
        goal_cell = Cell(goal[0], goal[1], goal[2])
        visited = [[[False for _ in range(len(grid))] for _ in range(len(grid[0]))] for _ in range(grid[0][0])]
    else:
        print("Invalid or obstructed start position: {start} or goal position: {goal}")
        return [], -1
    
    queue = [(start_cell, [])]
    visited = set()
    visited.add(tuple[start_cell])

    while queue:
        current_node, path = queue.pop(0)
        current_path = path + [tuple(current_node)]

        if tuple(current_node) == tuple(goal):
            print(f"Path found: {current_path}")
            return current_path, len(current_path) - 1
        
        for dx, dy, dz in neighbor_directions:
            nx, ny, nz = current_node.x + dx, current_node.y + dz, current_node.z + dy
            neighbor = is_valid_position_3d(grid, nx, nz, ny)

            if neighbor and not neighbor.is_obs and tuple(neighbor) not in visited:
                visited.add(tuple(neighbor))
                queue.append((neighbor, current_path))
    
    if path == []:
        print(f"No path found with BFS from {start} to {goal}")
    return [], -1