import heapq
import block_simulation.map_data as map_data
from colorama import Fore, init
init(autoreset=True)

class DStarLite:
    def __init__(self, grid, start, goal, structure_queue=None):
        self.grid = grid
        self.structure_queue = structure_queue
        self.cell_map = {}
        self.start = self.get_cell(start)
        self.goal = self.get_cell(goal, True)
        self.priority_queue = []
        self.km = 0 # changes in environment
        self.neighbors = map_data.set_neighbors(allow_large_build=True)
        self.goal.rhs = 0
        self.cell_map[self.goal.to_tuple()] = self.goal
        
        self.insert(self.goal, self.calculate_key(self.goal))
        
    def calculate_key(self, cell):
        """calculates the priority key for a cell"""
        g_rhs = min(cell.g, cell.rhs) #takes the minimum of the estimated cost and the one look ahead cost
        h = map_data.heuristic(self.start, cell)
        return(g_rhs + h + self.km + cell.cost, g_rhs)
    
    def insert(self, cell, key):
        heapq.heappush(self.priority_queue, (key, cell))
        
    def remove_from_queue(self, cell):
        """using the key (k) and its corresponding cell (c), remove it from the priority queue"""
        self.priority_queue = [(k, c) for (k, c) in self.priority_queue if c.to_tuple() != cell.to_tuple()]
        
    def get_cell(self, coords, is_goal=False):
        coords = tuple(coords)
        x, y, z = coords
        
        if coords not in self.cell_map:
            cell = map_data.create_cell(self.grid, coords)
            self.cell_map[coords] = cell
        else:
            cell = self.cell_map[coords]
            
        if not is_goal and self.structure_queue:
            cell.cost = 1
            for block in self.structure_queue:
                if coords == tuple(block):
                    cell.cost += 100 + 10 * z
                    break
            
        return cell
        
    def update_rhs(self, cell):
        """updates rhs and re-inserts it if needed"""
        if cell.to_tuple() != self.goal.to_tuple():
            min_rhs = float('inf')
            for dx, dy, dz in self.neighbors:
                nx, ny, nz = cell.x + dx, cell.y + dy, cell.z + dz
                neighbor_coord = nx, ny, nz
                if map_data.is_valid_position_3d(self.grid, (neighbor_coord)):
                    if ((self.grid[nx][ny][nz] == map_data.GridStatus.WALKABLE.value or self.grid[nx][ny][nz] == map_data.GridStatus.INCOMING_BLOCK.value)):
                        neighbor = self.get_cell(neighbor_coord)
                        min_rhs = min(min_rhs, neighbor.g + neighbor.cost)
            cell.rhs = min_rhs
        self.remove_from_queue(cell)
        if cell.g != cell.rhs:
            self.insert(cell, self.calculate_key(cell))
            
    def compute_shortest_path(self):
        while self.priority_queue and (self.priority_queue[0][0] < self.calculate_key(self.start) or self.start.rhs != self.start.g):
            k_old, cell = heapq.heappop(self.priority_queue) # pop out old key and corresponding cell
            k_new = self.calculate_key(cell)
            
            if k_old < k_new:
                self.insert(cell, k_new)
            elif cell.g > cell.rhs:
                cell.g = cell.rhs
                for dx, dy, dz in self.neighbors:
                    nx, ny, nz = cell.x + dx, cell.y + dy, cell.z + dz
                    neighbor_coord = nx, ny, nz

                    if map_data.is_valid_position_3d(self.grid, neighbor_coord):
                        neighbor = self.get_cell(neighbor_coord)

                        if self.structure_queue:
                            neighbor.cost = 1
                            for block in self.structure_queue:
                                if neighbor.to_tuple() == tuple(block):
                                    neighbor.cost += 100 + 10 * neighbor.z
                                    break

                        self.update_rhs(neighbor)
            else:
                cell.g = float('inf')
                self.update_rhs(cell)
                
                for dx, dy, dz in self.neighbors:
                    nx, ny, nz = cell.x + dx, cell.y + dy, cell.z + dz
                    neighbor_coord = nx, ny, nz

                    if map_data.is_valid_position_3d(self.grid, neighbor_coord):
                        neighbor = self.get_cell(neighbor_coord)

                        if self.structure_queue:
                            neighbor.cost = 1
                            for block in self.structure_queue:
                                if neighbor.to_tuple() == tuple(block):
                                    neighbor.cost += 100 + 10 * neighbor.z
                                    break

                        self.update_rhs(neighbor)

def find_path(grid, start, goal, iw_id, holding_block, structure_queue):
    """
    Perform D* Lite search in a 3D grid. 

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
        start (list): A list containing the (x, y, z) coordinate of the starting cell in a path.
                       The initial starting position can be configurable in config.py
        goal (list): A list containing the (x, y, z) coordinate of the ending cell in a path.
                      This typically is either the block depot or a block coordinate in the blueprint.
        iw_id (int): This inchworm's ID
        holding_block (bool): A flag that indicates if the inchworm is holding a block or not (which then changes the z).
        structure_queue (list): A list of the next blocks in the structure
    Returns:
        path (list[int]): A list of coordinates of the path.
    """
    start_status = (grid[start[0]][start[1]][start[2]])
    goal_status = (grid[goal[0]][goal[1]][goal[2]])
    print(Fore.MAGENTA + f"D* Lite called with start: {start} (status: {start_status}), goal: {goal} (status: {goal_status})")
    
    if not map_data.is_valid_start_goal_3d(grid, start, goal, iw_id):
        raise RuntimeError(f"Invalid start {start} or goal {goal} position")    
    d_star = DStarLite(grid, start, goal, structure_queue) # snapshot of what we have searched and found
    d_star.compute_shortest_path()
    current_cell = d_star.start
    
    while current_cell.to_tuple() != d_star.goal.to_tuple(): # Explore frontier 
        min_cost = float('inf')
        next_cell = None
        
        for dx, dy, dz in d_star.neighbors:
            nx, ny, nz = current_cell.x + dx, current_cell.y + dy, current_cell.z + dz
            neighbor_coord = nx, ny, nz
            if map_data.is_valid_position_3d(grid, (neighbor_coord)):
                if ((grid[nx][ny][nz] == map_data.GridStatus.WALKABLE.value or 
                     grid[nx][ny][nz-1] == map_data.GridStatus.INCOMING_BLOCK.value or
                     iw_id == map_data.GridStatus.which_inchworm(grid[nx][ny][nz]))):
                    neighbor = d_star.get_cell(neighbor_coord)
                    # print(Fore.YELLOW + f"Evaluating neighbor {neighbor_coord}: g={neighbor.g}, cost={neighbor.cost}, total={neighbor.g + neighbor.cost}")
                    if neighbor.g + neighbor.cost < min_cost:
                        min_cost = neighbor.g + neighbor.cost
                        next_cell = neighbor
        if next_cell is None:
            print(Fore.MAGENTA + f"No path found with D* Lite >:(")
            return []

        next_cell.parent = current_cell
        current_cell = next_cell
        
    path = map_data.reverse_path_3d(current_cell, holding_block)
    print(Fore.MAGENTA + f"Path found: {path}")
    return path