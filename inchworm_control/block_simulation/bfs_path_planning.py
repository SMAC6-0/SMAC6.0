import map_data
from colorama import Fore, init
init(autoreset=True)

def find_path(grid, start, goal, iw_id, holding_block) -> list[int]:
    """
    Perform modified BFS in a 3D grid.
    
    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
        start (list): A list containing the (x, y, z) coordinate of the starting cell in a path.
                       The initial starting position can be configurable in config.py
        goal (list): A list containing the (x, y, z) coordinate of the ending cell in a path.
                      This typically is either the block depot or a block coordinate in the blueprint.
        iw_id (int): This inchworm's ID
        holding_block (bool): A flag that indicates if the inchworm is holding a block or not (which then changes the z).
    Returns:
        path (list[int]): A list of coordinates of the path.
    """
    start_status = (grid[start[0]][start[1]][start[2]])
    goal_status = (grid[goal[0]][goal[1]][goal[2]])
    print(Fore.MAGENTA + f"BFS called with start: {start} (status: {start_status}), goal: {goal} (status: {goal_status})")
    
    neighbor_directions = map_data.set_neighbors(allow_large_build=True)
        
    if map_data.is_valid_start_goal_3d(grid, start, goal, iw_id):
        goal_cell, visited, frontier = map_data.start_bfs_3d(grid, start, goal)
    else:
        raise RuntimeError(f"Invalid start {start} or goal {goal} position")

    # if holding_block:
    #     frontier[0].z -= 1
    
    while frontier: # Explore frontier 
        current_cell = frontier.pop(0)

        if map_data.is_goal_reached_3d(current_cell, goal_cell): # If goal reached, finish exploring frontier 
            # map_data.handle_side_step(grid, current_cell, goal_cell, iw_id, holding_block)
            path = map_data.reverse_path_3d(current_cell, holding_block)
            print(Fore.MAGENTA + f"Path found: {path}")
            return path
        
        # Search for valid cells in the grid and add them to the frontier 
        for dx, dy, dz in neighbor_directions:
            nx, ny, nz = current_cell.x + dx, current_cell.y + dy, current_cell.z + dz
            neighbor_coord = nx, ny, nz
            if map_data.is_valid_position_3d(grid, (neighbor_coord)):
                if ((grid[nx][ny][nz] == map_data.GridStatus.WALKABLE.value or 
                     grid[nx][ny][nz] == map_data.GridStatus.INCOMING_BLOCK.value or
                     iw_id == map_data.GridStatus.which_inchworm(grid[nx][ny][nz])) and 
                     not visited[nx][ny][nz]):
                    visited[nx][ny][nz] = True
                    neighbor = map_data.create_cell(grid, neighbor_coord)
                    neighbor.parent = current_cell
                    frontier.append(neighbor)
    
    print(Fore.MAGENTA + f"No path found with BFS :(")#from {start_status} {start} to {goal_status} {goal}")
    return []