import numpy as np

# Length (X-axis): Determined by the number of columns in a row (6).
# Width (Y-axis): Determined by the number of rows in a layer (5).
# Depth (Z-axis): Determined by the number of layers (3).

sample_map = [ [[0, 0, 0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]], 
               [[0, 0, 0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]], 
               [[0, 0, 1, 0, 0, 0, 0, 0], [1, 1, 0, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]], 
               [[0, 0, 0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]], 
               [[0, 0, 0, 0, 3, 0, 0, 0], [1, 1, 1, 1, 0, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]], 
               [[0, 0, 0, 0, 0, 1, 0, 0], [1, 1, 1, 1, 1, 0, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]], 
               [[0, 0, 0, 0, 0, 1, 0, 0], [1, 1, 1, 1, 1, 0, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]]]

sample_final = [[[0, 0, 0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]],
                [[0, 0, 0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]], 
                [[0, 0, 1, 0, 0, 0, 0, 0], [1, 1, 0, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]],  
                [[0, 0, 0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]], 
                [[0, 0, 0, 0, 3, 0, 0, 0], [1, 1, 1, 1, 0, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]],  
                [[0, 0, 0, 0, 0, 1, 0, 0], [1, 1, 1, 1, 1, 0, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]],  
                [[0, 0, 0, 0, 0, 1, 0, 0], [1, 1, 1, 1, 1, 0, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1]]]
def blueprint(curr_map, final_map) -> list:
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
            print("map dims: ", curr_map.shape[0], curr_map.shape[1],curr_map.shape[2])
            print(curr_map[4,0,4])
            for y in range(curr_map.shape[0]): #iterate 0-7
                for z in range(curr_map.shape[1]): #iterate 0-7
                   for x in range((curr_map.shape[2])): #iterate 0-7
                        if curr_map[y, z, x] == 1 and curr_map[y, z, x] != final_map[y, z, x]:
                            return [y, z, x] # eventually, should return as x, y, z
    return [-9, -9, -9]

print(blueprint(sample_map, sample_final))