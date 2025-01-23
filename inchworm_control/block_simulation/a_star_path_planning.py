import heapq
from map_data import *

def heuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z) # manhattan

def a_star_search(grid, start, goal, prioritize_vertical=False):
    """
    Perform A* search in a 3D grid. 

    Args:
        grid (int): The size of the workspace, as a grid.
        start (int, int, int): The x, z, and y coordinate of the starting location.
        goal (int, int, int): The x, z, and y coordinate of the ending location.
        prioritize_vertical (boolean): A flag indicating if vertical neighbors are prioritized.
    Returns:
        path_coords (list): A list of A* path coordinates
        num_steps (int): The total number of steps taken by the inchworm
    """
    # Depending on prioritize_vertical, sets either the vertical (z) or the horizontal plane (x & y) as the first number of neighbors
    neighbor_directions = set_neighbors(prioritize_vertical)

    start_node = is_valid_position_3d(grid, *start)
    goal_node = is_valid_position_3d(grid, *goal)
    
    if not start_node or start_node.is_obs:
        print("Invalid or obstructed start position.")
        return [], -1
    if not goal_node or goal_node.is_obs:
        print("Invalid or obstructed goal position.")
        return [], -1
    
    start_node.g = 0
    start_node.h = heuristic(start_node, goal_node)
    start_node.f = start_node.g + start_node.h

    # priority queue for open nodes and list of visited nodes
    open_list = []
    heapq.heappush(open_list, (start_node.f, start_node))
    visited = set()

    while open_list:
        _, current_node = heapq.heappop(open_list)

        # if goal is reached, reconstruct path w/ parent pointers
        if (current_node.x, current_node.y, current_node.z) == (goal_node.x, goal_node.y, goal_node.z):
            return rework_path_3d(current_node, False)
        
        visited.add(current_node.x, current_node.y, current_node.z)

        for dx, dy, dz in neighbor_directions:
            nx, ny, nz = current_node.x + dx, current_node.y + dz, current_node.z + dy
            neighbor = is_valid_position_3d(grid, nx, nz, ny)

            if neighbor and not neighbor.is_obs and tuple(neighbor) not in visited:
                tentative_g = current_node.g + 1
                if (tentative_g < neighbor.g or tuple(neighbor) not in [tuple(n) for n in open_list]):
                    neighbor.g = tentative_g
                    neighbor.h = heuristic(neighbor, goal_node)
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.parent = current_node
                    heapq.heappush(open_list, neighbor)

    print("No path found with A*")
    return [], -1