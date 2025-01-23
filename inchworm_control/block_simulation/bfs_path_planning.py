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
        inchworm_id (int): An ID that identifies which inchworm grid, start, and goal is being taken in.
        prioritize_vertical (boolean): A flag that determines if vertical neighbors are prioritized (scaling walls).
    Returns:
        
    """
    print(f"BFS called with start: {start}, goal: {goal}, prioritize_vertical: {prioritize_vertical}")
    
    neighbor_directions = set_neighbors(prioritize_vertical)
    
    if is_valid_start_goal_3d(grid, *start, *goal):
        goal_cell, visited, queue, steps = start_search_3d(grid, start, goal)
    else:
        print("Invalid or obstructed start position: {start} or goal position: {goal}")
        return [], -1

    while queue:
        current_cell = queue.pop(0)
        steps += 1

        if is_goal_reached_3d(current_cell, goal_cell):
            path = rework_path_3d(current_cell, holding_block)
            print(f"Path found: {path}")
            return path, steps
        
        for dx, dy, dz in neighbor_directions:
            nx, ny, nz = current_cell.x + dx, current_cell.y + dz, current_cell.z + dy
            neighborCoord = nx, ny, nz
            if is_valid_position_3d(grid, (neighborCoord)) and not grid[nx][nz][ny] and not visited[nx][nz][ny]:
                visited[nx][nz][ny] = True
                neighbor = create_cell(grid, neighborCoord)
                neighbor.parent = current_cell
                queue.append(neighbor)
    
    print(f"No path found with BFS from {start} to {goal}")
    return [], -1