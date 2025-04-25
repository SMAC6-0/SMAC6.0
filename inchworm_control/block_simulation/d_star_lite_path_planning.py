import heapq
import map_data
from colorama import Fore, init
init(autoreset=True)

class DStarLite:
    def __init__(self, grid, start, goal, structure_queue=None, leading_foot=None, bypass_flag=False):
        self.grid = grid
        self.structure_queue = structure_queue
        self.bypass_flag = bypass_flag
        self.leading_foot = leading_foot
        self.lagging_foot = start
        self.neighbors = map_data.set_neighbors(allow_large_build=True)
        
        # instantiate
        self.cell_map = {}
        self.priority_queue = []
        self.km = 0 # changes in environment
        
        # find start, goal,
        self.goal = self.get_cell(goal, is_goal=True)
        self.start = self.get_cell(start, is_start=True)
        
        # cost mapping
        self.goal.rhs = 0
        # self.goal.g = self.start.rhs = self.start.g = float('inf')
        self.cell_map[self.start.to_tuple()] = self.start
        self.cell_map[self.goal.to_tuple()] = self.goal

        self.insert(self.goal, self.calculate_key(self.goal))

        for dx, dy, dz in self.neighbors:
            nx, ny, nz = self.start.x + dx, self.start.y + dy, self.start.z + dz
            neighbor_coord = nx, ny, nz
            if map_data.is_valid_position_3d(self.grid, neighbor_coord):
                neighbor = self.get_cell(neighbor_coord)
                self.update_rhs(neighbor)
        
    def calculate_key(self, cell):
        """calculates the priority key for a cell"""
        g_rhs = min(cell.g, cell.rhs) # takes the minimum of the estimated cost and the one look ahead cost
        h = map_data.heuristic(self.start, cell)
        return(g_rhs + h + self.km, g_rhs)
    
    def insert(self, cell, key):
        heapq.heappush(self.priority_queue, (key, cell))
        
    def remove(self, cell):
        """using the key (k) and its corresponding cell (c), remove it from the priority queue"""
        self.priority_queue = [(k, c) for (k, c) in self.priority_queue if c.to_tuple() != cell.to_tuple()]
        heapq.heapify(self.priority_queue)
        
    def get_cell(self, coords, is_start=False, is_goal=False):
        coords = tuple(coords)
        x, y, z = coords
        
        if coords not in self.cell_map:
            cell = map_data.create_cell(self.grid, coords)
            cell.cost = 1
            
            if not is_goal and self.structure_queue: # avoid structure but not goal
                for bx, by, bz in self.structure_queue:
                    for nx, ny, nz in self.neighbors:
                        block_neighbor = bx + nx, by + ny, bz + nz
                        if coords == block_neighbor and not is_start:
                            cell.cost += 5 * z
                            break
            if self.bypass_flag and coords == self.lagging_foot or coords == self.leading_foot: # penalize both feet locations to find new location
                cell.cost = 999
            self.cell_map[coords] = cell
        else:
            cell = self.cell_map[coords]
            if is_start or is_goal:
                cell.cost = 1
        return cell
        
    def update_rhs(self, cell):
        """updates rhs and re-inserts it if needed"""
        if cell.to_tuple() != self.goal.to_tuple():
            min_rhs = float('inf')
            for dx, dy, dz in self.neighbors:
                nx, ny, nz = cell.x + dx, cell.y + dy, cell.z + dz
                neighbor_coord = nx, ny, nz
                if map_data.is_valid_position_3d(self.grid, neighbor_coord):
                    if ((self.grid[nx][ny][nz] == map_data.GridStatus.WALKABLE.value or
                         self.grid[nx][ny][nz] == map_data.GridStatus.INCOMING_BLOCK.value or
                         neighbor_coord == self.goal.to_tuple())):
                        neighbor = self.get_cell(neighbor_coord)
                        min_rhs = min(min_rhs, neighbor.g + neighbor.cost)
            cell.rhs = min_rhs

        # if cell in self.priority_queue:
        self.remove(cell)
            
        if cell.g != cell.rhs:
            self.insert(cell, self.calculate_key(cell))
            
    def compute_shortest_path(self):
        iteration = 0
        while self.priority_queue and (self.priority_queue[0][0] < self.calculate_key(self.start) or self.start.rhs != self.start.g):
            iteration += 1
            # print(f"\n--- Iteration {iteration} ---")
            # print(f"Start g: {self.start.g}, rhs: {self.start.rhs}")
            # print(f"Top key: {self.priority_queue[0][0]}, Start key: {self.calculate_key(self.start)}")
            
            # if iteration > 1000:
            #     print("loop de loop")
            #     break
            
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
                        self.update_rhs(neighbor)
            else:
                cell.g = float('inf')
                self.update_rhs(cell)
                for dx, dy, dz in self.neighbors:
                    nx, ny, nz = cell.x + dx, cell.y + dy, cell.z + dz
                    neighbor_coord = nx, ny, nz
                    if map_data.is_valid_position_3d(self.grid, neighbor_coord):
                        neighbor = self.get_cell(neighbor_coord)
                        self.update_rhs(neighbor)

def find_path(grid, leading_foot_loc, lagging_foot_loc, goal, iw_id, holding_block, structure_queue, bypass_flag):
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
    start = lagging_foot_loc
    start_status = (grid[start[0]][start[1]][start[2]])
    goal_status = (grid[goal[0]][goal[1]][goal[2]])
    print(Fore.MAGENTA + f"D* Lite called with start: {start} (status: {start_status}), goal: {goal} (status: {goal_status})")
    below_goal_status = (grid[goal[0]][goal[1]][goal[2]-1])
    # print(Fore.CYAN + f"below goal: {[goal[0], goal[1], goal[2] - 1]} (status: {below_goal_status})")
    
    if not bypass_flag:
        if not map_data.is_valid_start_goal_3d(grid, start, goal, iw_id):
            raise RuntimeError(f"Invalid start {start} or goal {goal} position")
        
    d_star = DStarLite(grid, start, goal, structure_queue, leading_foot_loc, bypass_flag) # snapshot of what we have searched and found
    # if goal == [3, 3, 1]:
    #     print("before d star stuff")
    d_star.compute_shortest_path()
    # if goal == [3, 3, 1]:
    #     print("after compute_shortest_path")
    current_cell = d_star.start
    # if goal == [3, 3, 1]:
    #     print(f"before while loop. current cell: {current_cell.to_tuple()}. goal: {d_star.goal.to_tuple()}")
    while current_cell.to_tuple() != d_star.goal.to_tuple(): # Explore frontier 
        min_cost = float('inf')
        next_cell = None
        # if goal == [3, 3, 1]:
            # print("in w/hile loop")
        for dx, dy, dz in d_star.neighbors:
            nx, ny, nz = current_cell.x + dx, current_cell.y + dy, current_cell.z + dz
            neighbor_coord = nx, ny, nz
            if map_data.is_valid_position_3d(grid, (neighbor_coord)):
                if ((grid[nx][ny][nz] == map_data.GridStatus.WALKABLE.value or 
                     grid[nx][ny][nz] == map_data.GridStatus.INCOMING_BLOCK.value or
                     iw_id == map_data.GridStatus.which_inchworm(grid[nx][ny][nz]))):
                    neighbor = d_star.get_cell(neighbor_coord)
                    total_cost = neighbor.rhs#d_star.calculate_key(neighbor)[0]
                    # if goal == [3, 3, 1]:
                    #     print(f"valid position, ok status. total cost: {total_cost}, min_cost {min_cost}")
                    if total_cost < min_cost:
                        min_cost = total_cost
                        next_cell = neighbor
                        # if goal == [3, 3, 1]:
                        #     print(Fore.YELLOW + f"Evaluating neighbor {neighbor_coord} (status: {grid[nx][ny][nz]}): g={neighbor.g}, cost={neighbor.cost}, total={neighbor.g + neighbor.cost}-------------------------------")
                        #     print(Fore.LIGHTRED_EX + f"walkable? {grid[nx][ny][nz] == map_data.GridStatus.WALKABLE.value}, incoming? {grid[nx][ny][nz] == map_data.GridStatus.INCOMING_BLOCK.value}, my path? {iw_id == map_data.GridStatus.which_inchworm(grid[nx][ny][nz])}")
        # if goal == [3, 3, 1]:
        #     print("after while loop ")
        if next_cell is None:
            # print(Fore.MAGENTA + f"mappity map: \n{grid}")
            print(Fore.MAGENTA + f"No path found with D* Lite >:(")
            return []

        next_cell.parent = current_cell
    
        current_cell = next_cell
        
    path = map_data.reverse_path_3d(current_cell, holding_block)
    print(Fore.MAGENTA + f"Path found: {path}")
    return path