import numpy as np

# Length (X-axis): Determined by the number of columns in a row (6).
# Width (Y-axis): Determined by the number of rows in a layer (5).
# Depth (Z-axis): Determined by the number of layers (3).


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
            for z in range(curr_map.shape[0]): #iterate 0-2 
                for y in range(curr_map.shape[1]): #iterate 0-4
                   for x in range((curr_map.shape[2])): #iterate 0-5 
                        if curr_map[z, x, y] == 1 and curr_map[z, x, y] != final_map[z, x, y]:
                            return [z, y, x]
    return [-9, -9, -9]