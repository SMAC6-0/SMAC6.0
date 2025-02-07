import map_data
import config

def find_path(grid, start, goal, holding_block) -> list[int]:
    """
    Perform modified BFS in a 3D grid.
    
    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
        start (tuple): A tuple containing the (x, z, y) coordinate of the starting cell in a path.
                       The initial starting position can be configurable in config.py
        goal (tuple): A tuple containing the (x, z, y) coordinate of the ending cell in a path.
                      This typically is either the block depot or a block coordinate in the blueprint.
        holding_block (bool): A flag that indicates if the inchworm is holding a block or not (which then changes the z).
    Returns:
        path (list[int]): A list of coordinates of the path.
    """
    print(f"BFS called with start: {start}, goal: {goal}")
    
    neighbor_directions = map_data.set_neighbors()
    
    if map_data.is_valid_start_goal_3d(grid, start, goal):
        goal_cell, visited, queue = map_data.start_search_3d(grid, start, goal)
    else:
        raise RuntimeError(f"Invalid start {start} or goal {goal} position\n",
                           f"Start Walkable? {grid[start[0]][start[1]][start[2]] == 0}\n",
                           f"Goal Walkable? {grid[goal[0]][goal[1]][goal[2]] == 0}")

    if holding_block:
        queue[0].z -= 1
    
    while queue:
        current_cell = queue.pop(0)

        if map_data.is_goal_reached_3d(current_cell, goal_cell):
            path = map_data.reverse_path_3d(current_cell, holding_block)
            print(f"Path found: {path}")
            return path
        
        for dx, dz, dy in neighbor_directions:
            nx, nz, ny = current_cell.x + dx, current_cell.z + dz, current_cell.y + dy
            neighbor_coord = nx, nz, ny
            if map_data.is_valid_position_3d(grid, (neighbor_coord)) and (grid[nx][nz][ny] == 0 or grid[nx][nz][ny] == 2) and not visited[nx][nz][ny]:
                visited[nx][nz][ny] = True
                neighbor = map_data.create_cell(grid, neighbor_coord)
                neighbor.parent = current_cell
                queue.append(neighbor)
    
    print(f"No path found with BFS from {start} to {goal}")
    return []