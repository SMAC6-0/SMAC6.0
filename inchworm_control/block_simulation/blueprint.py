import numpy as np
from collections import defaultdict
from config import SEED_BK

# x: row in array (7 rows)
# y: layer (6 layers)
# z: col in array (8 cols)
sample_map = [[[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]],  
              [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]],  
              [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]],  
              [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
              [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [1, 1, 0, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [3, 0, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]],  
              [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
              [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]]]

sample_final = [[[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
                [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
                [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
                [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
                [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [1, 0, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [3, 0, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
                [[1, 0, 1, 1, 1, 1, 1, 1], [1, 0, 1, 1, 1, 1, 1, 1], [1, 0, 1, 1, 1, 1, 1, 1], [1, 0, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
                [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [1, 0, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]]]

sample_stacked_final = [[[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
                        [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
                        [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
                        [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
                        [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [1, 1, 0, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [3, 0, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], # stacked block of 3
                        [[1, 0, 1, 1, 1, 1, 1, 1], [1, 0, 1, 1, 1, 1, 1, 1], [1, 0, 1, 1, 1, 1, 1, 1], [1, 0, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]], 
                        [[0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [1, 0, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1]]]


class BlueprintAlgorithm:
    def __init__(self, repeat_threshold=2):
        self._last_block = None
        self._repeat_count = 0
        self._repeat_threshold = repeat_threshold
        self._priority_queue = []
        
    def blueprint(self, curr_map, final_map) -> list:            
        curr_map = np.array(curr_map)
        final_map = np.array(final_map)
        # arrays are not the same size
        if curr_map.size != final_map.size:
            print("Arrays don't match sizes") 
            return [-9,-9,-9] # error value

        elif curr_map.size == final_map.size:
            # The structure is complete
            if np.array_equal(curr_map, final_map):
                return [-1,-1,-1]
            else: 
                # TODO: implement prioritization of found structures
                # print("map dims: ", curr_map.shape[0], curr_map.shape[1],curr_map.shape[2])
                # print(curr_map[4,4,0])
                self._priority_queue = []
                for z in range((curr_map.shape[2])): #iterate 0-7
                    for x in range(curr_map.shape[0]): #iterate 0-7
                        for y in range(curr_map.shape[1]): #iterate 0-7
                            if curr_map[x, y, z] != 2: 
                                is_different = curr_map[x, y, z] != final_map[x, y, z]
                                final_is_walkable = final_map[x, y, z] == 0
                                lowest_z = z

                                
                                for zz in range((curr_map.shape[2])):
                                    curr_is_walkable = curr_map[x, y, zz] == 0
                                    if curr_is_walkable:
                                        lowest_z = zz
                                
                                if is_different and final_is_walkable:
                                    for zz in range(lowest_z + 1, z + 1):
                                        self._priority_queue.append((x, y, zz))
                         
                if not self._priority_queue:
                    return [-9, -9, -9]
                           
                z_groups = defaultdict(list)
                # prioritize lower z
                for x, y, z in self._priority_queue:
                    z_groups[z].append((x, y))

                # prioritize from seed block outwards
                seed_x, seed_y, seed_z = SEED_BK

                def seed_distance(x, y):
                    return abs(x - seed_x) ** 2 + (y - seed_y) ** 2

                sorted_coords = []
                for z in sorted(z_groups.keys()):
                    sorted_xy = sorted(z_groups[z], key=lambda xy: seed_distance(xy[0], xy[1]))
                    for x, y in sorted_xy:
                        sorted_coords.append((x, y, z))
                        
                # print(sorted_coords)                
                next_block = sorted_coords[0]

                if self._last_block == next_block:
                    self._count += 1
                else:
                    self._count = 0
                    self._last_block = next_block

                if self._count >= self._repeat_threshold and len(sorted_coords) > 1:
                    print(f"repeat threshold exceeded {self._repeat_threshold}, new block time!!!")
                    next_block = sorted_coords[1]
                    self._last_block = next_block
                    self._count = 0
                    print(f"block: {self._last_block} count: {self._count}")
                return list(next_block)
        return [-9, -9, -9]

# bp = BlueprintAlgorithm()
# print(bp.blueprint(sample_map, sample_stacked_final))
# print(bp.blueprint(sample_map, sample_stacked_final))
# print(bp.blueprint(sample_map, sample_stacked_final))