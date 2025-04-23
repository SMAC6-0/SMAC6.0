import numpy as np
from collections import defaultdict
from block_simulation.config import SEED_BK
import json

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
        
build_queue = []
last_block = None
repeat_count = 0

def seed_distance(x, y):
    return abs(x - SEED_BK[0]) ** 2 + (y - SEED_BK[1]) ** 2

def blueprint(curr_map, final_map, repeat_threshold=2) -> list: 
    global build_queue, last_block, repeat_count
               
    curr_map = np.array(curr_map)
    final_map = np.array(final_map)
    map_complete = True

    if curr_map.shape != final_map.shape:
        print("Arrays don't match shape") 
        return [-9,-9,-9] # error value

    elif curr_map.size == final_map.size:
        # The structure is complete
        if np.array_equal(curr_map, final_map):
            return [-1,-1,-1], build_queue
        
        # TODO: implement prioritization of found structures
        # print("map dims: ", curr_map.shape[0], curr_map.shape[1],curr_map.shape[2])
        # print(curr_map[4,4,0])
        for z in range((curr_map.shape[2])): #iterate 0-7
            for x in range(curr_map.shape[0]): #iterate 0-7
                for y in range(curr_map.shape[1]): #iterate 0-7
                        is_different = curr_map[x, y, z] != final_map[x, y, z]
                        final_is_walkable = final_map[x, y, z] == 0
                        lowest_z = z
                        
                        for zz in range((curr_map.shape[2])):
                            curr_is_walkable = curr_map[x, y, zz] == 0
                            if curr_is_walkable:
                                lowest_z = zz
                                break
                        
                        if is_different and final_is_walkable:
                            for zz in range(lowest_z + 1, z + 1):
                                curr_is_not_incoming = curr_map[x, y, zz] != 2
                                if curr_is_not_incoming and (x, y, zz) not in build_queue:
                                    build_queue.append((x, y, zz))
                        
        z_groups = defaultdict(list)
        # prioritize lower z
        for x, y, z in build_queue:
            z_groups[z].append((x, y))

        sorted_queue = []
        for z in sorted(z_groups.keys()):
            sorted_xy = sorted(z_groups[z], key=lambda xy: seed_distance(xy[0], xy[1]))
            for x, y in sorted_xy:
                sorted_queue.append((x, y, z))
                
        build_queue = sorted_queue
        
        if not build_queue:
            return [-9, -9, -9], build_queue
                
        # print(sorted_coords)                
        next_block = build_queue[0]

        if last_block == next_block:
            repeat_count += 1
        else:
            repeat_count = 0
            last_block = next_block

        if repeat_count >= repeat_threshold and len(build_queue) > 1:
            print(f"repeat threshold exceeded {repeat_threshold}, new block time!!!")
            build_queue.insert(1, build_queue.pop(0))
            next_block = build_queue[0]
            last_block = next_block
            repeat_count = 0
            print(f"block: {last_block} count: {repeat_count}")
        
        build_queue.pop(0) # get rid of next block for other iws
        # print(f"Selected block: {next_block}, Queue: {build_queue}")
        return list(next_block), build_queue
    return [-9, -9, -9]

# bp = BlueprintAlgorithm()
# print(bp.blueprint(sample_map, sample_stacked_final))
# print(bp.blueprint(sample_map, sample_stacked_final))
# print(bp.blueprint(sample_map, sample_stacked_final))