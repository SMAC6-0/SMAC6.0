import numpy as np

# Length (X-axis): Determined by the number of columns in a row (6).
# Width (Y-axis): Determined by the number of rows in a layer (5).
# Depth (Z-axis): Determined by the number of layers (3).
FINAL_MAP = np.array([
    [  # Z = 0
        [1, 0, 0, 0, 0, 0], # X row
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ],
    [  # Z = 1
        [1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ],
    [  # Z = 2
        [1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ]])


def blueprint(curr_map: np.array) -> tuple:
    # arrays are not the same size
    if curr_map.size != FINAL_MAP.size:
        print("Arrays don't match sizes") 
        return (-9,-9,-9) # error value

    elif curr_map.size == FINAL_MAP.size:
        # The structure is complete
        if np.array_equal(curr_map, FINAL_MAP):
            return (-1,-1,-1)
        else: 
            print(curr_map.shape)
            # TODO: implement prioritization of found structures
            for z in range(curr_map.shape[0]): #iterate 0-2
                for y in range(curr_map.shape[1]): #iterate 0-4
                   for x in range((curr_map.shape[2])): #iterate 0-5
                        if curr_map[z][y][x] == 0 and curr_map[z][y][x] != FINAL_MAP[z][y][x]:
                            print("X: ", x)
                            print("Y: ", y)
                            print("Z: ", z)

                            print(curr_map[z][x][y])
                            return (x, z, y)

                
                    
# testing purpose
if __name__ == "__main__":
    curr_map = np.array([
    [  # Z = 0
        # Y col
        [1, 0, 0, 0, 0, 0], # X row
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ],
    [  # Z = 1
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ],
    [  # Z = 2
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ]])
    print("From Blueprint", blueprint(curr_map))
