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
        
    if map_data.is_valid_start_goal_3d(grid, start, goal):
        goal_cell, visited, frontier = map_data.start_search_3d(grid, start, goal)
    else:
        raise RuntimeError(f"Invalid start {start} or goal {goal} position\n",
                           f"Start Walkable? {start_status == 0}\n",
                           f"Goal Walkable? {goal_status == 0}")

    # if holding_block:
    #     frontier[0].z -= 1
    
    while frontier: # Explore frontier 
        current_cell = frontier.pop(0)

        if map_data.is_goal_reached_3d(current_cell, goal_cell): # If goal reached, finish exploring frontier 

            # If goal is reached and a block is going to be placed, make an extra step to the side
            if holding_block: 
                goal_adjacent = current_cell.parent # This is the cell right next to the goal cell, the step right before the goal itself 
                if (goal_cell.z - goal_adjacent.z) > 1: # Only bother adding the step if this block is higher up
                    print(Fore.MAGENTA + f"Trying to add a pivot")
                    adjacent_neighbor_dirs = map_data.set_neighbors(allow_vertical=False, allow_vert_diagonal=False)
                    diagonal_neighbor_dirs = map_data.set_neighbors(allow_adjacent=False, allow_vertical=False, allow_vert_diagonal=False, allow_horz_diagonal=True)

                    ground_coord = [goal[0], goal[1], goal[2] - (goal_cell.z - goal_adjacent.z)] # Look for pivot steps on the same level as the inchworm would be before placement 
                    for dx, dy, dz in adjacent_neighbor_dirs: 
                        px, py, pz = goal_adjacent.x + dx, goal_adjacent.y + dy, goal_adjacent.z + dz # Examine potential side steps (adjacent to goal_adjacent)
                        pivot_coord = [px, py, pz]

                        # The step to the side should be diagonal from the goal 
                        # print(Fore.MAGENTA + f"Trying to find neighbors for {pivot_coord} and {ground_coord}")
                        # print(f"IW{iw_id}: value at {pivot_coord} is {grid[px][py][pz]}")
                        if (pivot_coord != ground_coord and
                            map_data.is_neighbor_of_cell(grid, pivot_coord, ground_coord, diagonal_neighbor_dirs) and 
                            (grid[px][py][pz] == map_data.GridStatus.WALKABLE.value or grid[px][py][pz] == (iw_id*map_data.GridStatus.INCHWORM_PATH.value))): 
                            # If a suitable location, add this step to the path
                            pivot_cell = map_data.create_cell(grid, pivot_coord)
                            pivot_cell.parent = goal_adjacent 
                            current_cell.parent = pivot_cell # Same as the changing the parent to reach the goal cell 
                            print(Fore.MAGENTA + f"Added a pivot cell at {pivot_coord}")
                            # break
                        else: 
                            print(Fore.MAGENTA + f"Failed to add a pivot cell at {pivot_coord}")
                    if not pivot_cell: # If no pivot cell was found
                        print(Fore.MAGENTA + f"No path found with BFS :(")#from {start_status} {start} to {goal_status} {goal}")
                        return []

            path = map_data.reverse_path_3d(current_cell, holding_block)
            print(Fore.MAGENTA + f"Path found: {path}")
            return path
        
        # Search for valid cells in the grid and add them to the frontier 
        for dx, dy, dz in neighbor_directions:
            nx, ny, nz = current_cell.x + dx, current_cell.y + dy, current_cell.z + dz
            neighbor_coord = nx, ny, nz
            if (map_data.is_valid_position_3d(grid, (neighbor_coord)) and 
                (grid[nx][ny][nz] == map_data.GridStatus.WALKABLE.value or grid[nx][ny][nz] == map_data.GridStatus.INCOMING_BLOCK.value)  
                and not visited[nx][ny][nz]):
                visited[nx][ny][nz] = True
                neighbor = map_data.create_cell(grid, neighbor_coord)
                neighbor.parent = current_cell
                frontier.append(neighbor)
    
    print(Fore.MAGENTA + f"No path found with BFS :(")#from {start_status} {start} to {goal_status} {goal}")
    return []