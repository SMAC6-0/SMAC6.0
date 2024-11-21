import copy
from config import BD_LOC
from map_data import *

def breadth_first_search(grid, start, goal, prioritize_vertical=False):
    """
    Perform modified BFS in a 3D grid.
    
    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
        start (tuple): 
    If prioritize_vertical is True, vertical neighbors are prioritized.
    """
    print(f"BFS called with start: {start}, goal: {goal}, prioritize_vertical: {prioritize_vertical}")
    
    neighbor_directions = set_neighbors(prioritize_vertical)
    start_node = is_valid_position_3d(grid, *start)
    goal_node = is_valid_position_3d(grid, *goal)
    
    if not start_node or start_node.is_obs:
        print("Invalid or obstructed start position: {start}")
        return [], -1
    if not goal_node or goal_node.is_obs:
        print("Invalid or obstructed goal position: {goal}")
        return [], -1
    
    queue = [(start_node, [])]
    visited = set()
    visited.add(tuple[start_node])

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