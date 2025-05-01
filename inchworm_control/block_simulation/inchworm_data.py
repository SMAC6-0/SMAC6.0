#!/usr/bin/env python3
from enum import Enum
import copy
from config import *
import map_data
import blueprint as bp
from time import sleep
import serial
import struct
import json
from colorama import Fore, init
import numpy as np
init(autoreset=True)
if not SIMULATION:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from actions_messages.action import Inchwormpath
    from actions_messages.msg import Step

###### UART stuff
UART_BAUD = 9600 # config

INCHWORM_MOVED = True # setting this to True so it can bypass all the movements for debugging 

test_path_delete = [[5, 1, 0], [4, 1, 0], [3, 1, 0], [2, 1, 0], [1, 1, 0],]

class UART_CODES(Enum):
    StartByte=0xAA 
    Initialization=0xFA
    BeingPlaced=0xFB 
    MapSnapshot=0xFC

    Changes=0xFE
    Failed=0xFF
    NewInchworm=0xEF

IW_identifier = 1 # this is the idenifier that goes infornt of the message to be sent to the block 


dummy_block_location = [1, 0, 1] # testing value #TODO: change this later

# Inchworm states
class IW_STATE(Enum):
    IDLE = 1 # added this incase we need to use it
    INITIALIZATION = 2
    PATH_PLANNING = 3
    TRAVELLING_TO_SUPPLY = 4
    TRANSPORTING_BLOCK = 5
    PLACING_BLOCK = 6
    ERROR = 7
    STRUCTURE_COMPLETE = 8
    YIELDING = 9

PATH_PLANNING_TIMER = 5
COMMUNICATION_TIMER = 0.5 # DO NOT CHANGE THIS ANY LESS THAN 0.4

lagging_transform = {
    InchwormOrientation.NORTH: lambda x, y, z: (x, y - 1, z),  
    InchwormOrientation.SOUTH: lambda x, y, z: (x, y + 1, z),
    InchwormOrientation.EAST: lambda x, y, z: (x - 1, y, z), 
    InchwormOrientation.WEST: lambda x, y, z: (x + 1, y, z) 
}

class Inchworm():
    next_id = 1
    inchworm_list = []
    
    def __init__(self, orientation, final_structure, location: tuple[int]=IW_1_LOC, holding_block=False):
        """
        Initialize one inchworm (abbreviated as IW) in the system.
        Args:
            id (int): This inchworm's ID number. Used to set paths in the map. 
            orientation (Enum): the direction that the IW's leading leg is facing, relative to the world grid's frame. 
            location (tuple[int]): the xzy location of the inchworm's leading foot. 
            holding_block (bool): True if the inchworm's leading foot is holding a block. 
        """
        # Essential information for IW to keep track of
        self.id = Inchworm.next_id
        self.orientation = orientation
        
        self.current_map = map_data.initialize_grid()
        self.final_structure = final_structure
        self.holding_block = holding_block

        # Path planning relevant vars
        self.paths = [] # the list of coords
        self.temp_path = []
        self.goal = [] # goal coord
        self.priority_snapshot = []
        self.num_steps = 0
        self.step_num = 1
        self.next_block_loc = [] # stores the location of next block
        # self.found_structures = []
        # self.misc_blocks = []
        self.iw_path_id = map_data.GridStatus.inchworm_path(self.id) 
        self.clear_path_com = [] # stores the list of path to send to the blocks to clear from the map 
        self.IW_message_counter = 1 # this is the messgae counter for sending data, IW's message counter increases

        # Leg locations for the inchworm. 
        self.leading_foot_loc = location
        self.lagging_foot_loc = list(lagging_transform[orientation](*self.leading_foot_loc))
        
        # pertaining to the state machine 
        self.prev_state = None
        self.state = IW_STATE.INITIALIZATION
        self.print_flag = True
        self.iw_reached_seed_block_flag = False # flag to make sure IW reached the seed block and can now request map
        
        Inchworm.next_id += 1
        Inchworm.inchworm_list.append(self)

        # UART stuff
        if not SIMULATION: 
            self.IW_SERIAL = serial.Serial ("/dev/ttyAMA0", 9600)    #Open port with baud rate
        else: 
            self.total_num_steps = 0
            # These are vars that the simulation uses to simulate each of the inchworm feet
            self.last_cell, self.last_bk_og_texture, self.last_cell_2, self.last_bk_og_texture_2, self.spawned, self.prev_held_block_loc = None, None, None, None, False, [0,0,0]

    def __del__(self):
        """
        Deletion of inchworm in the list of inchworms.
        """
        Inchworm.inchworm_list = [iw for iw in Inchworm.inchworm_list if iw.id != self.id]
    
    def dummy_IW_move(self):
        make_IW_move = input("Make IW move? (y/n) \n")
        if make_IW_move.lower() == 'y':
            self.get_next_step()
            return True
        elif make_IW_move.lower() == 'n':
            return False
        else:
            print(" Invalid input. Please answer with 'yes' or 'no'.")

    def plan_path(self, next_goal: tuple[int, int, int] = None, bypass_flag=False): 
        """ Plan path from current location to specified goal. 
            Path planning typically starts from the pivot foot to avoid unnecessary walking on the structure"""
        # print(Fore.MAGENTA + f"IW{self.id}, leading: {self.leading_foot_loc}, lagging foot loc: {self.lagging_foot_loc}")
        is_traveling = False # assumes that if not specified, objective is to travel, not place

        if next_goal == None:
            # print(Fore.MAGENTA + f"IW{self.id}: goal not given... finding goal now")            
            self.goal, self.priority_snapshot = bp.blueprint(self.current_map, self.final_structure) # gets goal from blueprint if none is given
            print(Fore.MAGENTA + f"IW{self.id}: goal: {self.goal}")
            if self.goal == [-1, -1, -1]:
                # print(Fore.MAGENTA + f"Other IWs finished the structure while IW{self.id} was looking for a path")
                return
            elif self.goal == [-9, -9, -9]:
                # print(Fore.MAGENTA + f"IW{self.id}: Erm... No goal was given... No structure was found...")
                path = []
                self.goal = self.leading_foot_loc
                return
        else:
            is_traveling = True
            self.goal = next_goal
            if self.goal != SEED_BK: 
                # print(Fore.MAGENTA + f"IW{self.id}: Goal is not seed block. Setting IW's goal to be incoming block")
                self.current_map = map_data.update_grid_status(self.current_map, self.goal, map_data.GridStatus.INCOMING_BLOCK.value) # updates map for next_goal to be incoming_block

        # print(Fore.MAGENTA + f"IW{self.id}'s current_map: \n{self.current_map}")
        # print(Fore.MAGENTA + f"(PP) final_map: \n{self.final_structure}")

        try: 
            step_instructions, steps, path = [], [], []
            if is_traveling or self.holding_block:
                # Find one path, to travel to the specified goal
                path, steps, new_orientation = map_data.initiate_find_path(self.current_map, self.leading_foot_loc, self.lagging_foot_loc, self.goal, self.orientation, self.holding_block, self.id, self.priority_snapshot, bypass_flag)
            else:
                # Find path to block depot 
                bd_path, bd_steps, new_orientation = map_data.initiate_find_path(self.current_map, self.leading_foot_loc, self.lagging_foot_loc, BD_1_LOC, self.orientation, self.holding_block, self.id, self.priority_snapshot, bypass_flag)
                # self.holding_block = True

                # If it doesn't find a path to the supply depot, just return, don't bother trying to path plan further
                if bd_path == []: 
                    return
                
                if [self.goal[0], self.goal[1], self.goal[2]+1] != SEED_BK:
                    self.next_block_loc = [self.goal[0], self.goal[1], self.goal[2]-1]
                    # print(Fore.MAGENTA + f"IW{self.id}: Setting IW's goal to be incoming block")
                    self.current_map = map_data.update_grid_status(self.current_map, self.goal, map_data.GridStatus.INCOMING_BLOCK.value) # updates map for next_goal to be incoming_block
                
                # Find path to where the next block will be placed
                goal_path, goal_steps, new_orientation = map_data.initiate_find_path(self.current_map, bd_path[-1], bd_path[-2], self.goal, new_orientation, self.holding_block, self.id, self.priority_snapshot, bypass_flag)
                self.holding_block = False
                if goal_path == []: 
                    return
                goal_path.pop(0) # Remove repeat coord
                
                # combines start to block depot and block depot to goal
                steps += bd_steps
                steps += goal_steps
                path += bd_path
                path += goal_path
            
            # Update inchworm path & corresponding steps to travel that path
            step_instructions += steps
            self.paths += path
            self.num_steps = len(step_instructions)

            # Update the inchworm's internal map with the step it will take 
            self.current_map = map_data.set_inchworm_path_to_grid(self.current_map, self.paths, self.id) # Update IW's map with the path
            
            # Save the step instructions 
            self.step_instructions = step_instructions
            print(Fore.BLUE + f" IW{self.id}: step instructions: {self.step_instructions}")
            x, y,z = self.goal
            print( f" IW{self.id}: chosen goal {self.goal} has status {self.current_map[x][y][z]}. underneath, {[x, y, z-1]}, has status {self.current_map[x][y][z-1]}")
            
        except RuntimeError as e:
            print(Fore.MAGENTA + f" IW{self.id}: {e}. No path found, try again later.")
            return
    
    def get_next_step(self):
        """Returns the leading foot location as is used for the simulation"""
        if self.step_num > self.num_steps: 
            ValueError(Fore.BLUE + f"Erm we're on step {self.step_num} but there should be {self.num_steps} steps")
        else: 
            step_type = ""
            if self.step_instructions:
                step = self.step_instructions[self.step_num-1]
                print(Fore.BLUE + f"IW{self.id}: Next step: {step}. This is step {self.step_num}/{self.num_steps} for path of length {len(self.paths)}")
                step_type = step[0]
                step_change = step[1]
                orientation_transforms = {
                    InchwormOrientation.NORTH: lambda x, y, z: [-y,  x, z],  
                    InchwormOrientation.SOUTH: lambda x, y, z: [ y, -x, z],
                    InchwormOrientation.EAST:  lambda x, y, z: [ x,  y, z],  
                    InchwormOrientation.WEST:  lambda x, y, z: [-x, -y, z],  
                }
                transform = orientation_transforms[self.orientation]
                transformed_vector = transform(*step_change)
                transformed_vector = list(map(int, transformed_vector))

                prev_leading = self.leading_foot_loc
                # print(f"change in world frame: {transformed_vector}")
                self.leading_foot_loc = [self.leading_foot_loc[i] + transformed_vector[i] for i in range(len(transformed_vector))]  
                # Update Inchworm Orientation with each step
                self.orientation = map_data.get_orientation(step_change, self.orientation)

                # If the IW is NOT placing a block right now
                if "PLACE" not in step_type:
                    self.lagging_foot_loc = list(lagging_transform[self.orientation](*self.leading_foot_loc))
                    # If the inchworm is stepping up 
                    # if step_change[2] > 0: 
                    #     self.lagging_foot_loc[2] = self.leading_foot_loc[2] - prev_leading[2]
                    # elif step_change[2] < 0: 
                    #     self.lagging_foot_loc[2] = prev_leading[2] - self.leading_foot_loc[2] 
                    if step_change[2] > 0: 
                        print(f"{prev_leading[2]}")
                        print(f"{self.leading_foot_loc[2]}")
                        # self.lagging_foot_loc[2] = self.leading_foot_loc[2] - 1
                        for zz in range(GRID_HEIGHT):
                            curr_is_walkable = self.current_map[self.lagging_foot_loc[0]][self.lagging_foot_loc[1]][zz]
                            if curr_is_walkable:
                                lowest_z = zz
                                break
                        self.lagging_foot_loc[2] = lowest_z
                elif "PLACE" in step_type:
                    for zz in range(GRID_HEIGHT):
                        curr_is_walkable = self.current_map[self.lagging_foot_loc[0]][self.lagging_foot_loc[1]][zz]
                        if curr_is_walkable:
                            lowest_z = zz
                            break
                    self.lagging_foot_loc[2] = lowest_z
                self.step_num += 1
       
            x, y, z = self.leading_foot_loc
            
            # If the IW is grabbing a block, holding_block becomes true
            if "GRAB" in step_type:
                self.holding_block = True
            # Then, if placing a block, IW is no longer holding the block
            elif "PLACE" in step_type: #self.holding_block & ([x, y, z] == [self.goal[0], self.goal[1], self.goal[2]-1]):
                self.holding_block = False
            
            if self.holding_block and [x, y, z] != self.goal:
                z = z + 1
            print( f" IW{self.id}: foot locs: {[x, y, z]}, {self.lagging_foot_loc}")
            return x, y, z
    
    def get_total_inchworms(cls):
        """
        Returns the total number of inchworms in the system
        """
        return len(cls.inchworm_list)
        
    def reset_inchworms(cls):
        cls.next_id = 1
        cls.inchworm_list.clear()
        
    def find_nearest_structure(self):
        struct = []
        for x in range(len(self.current_map)):
            for y in range(len(self.current_map[0])):
                for z in range(len(self.current_map[0][0])):
                    if map_data.is_valid_position_3d(self.current_map, (x, y, z)) and map_data.is_valid_position_3d(self.current_map, (x, y, z + 1)):
                        if (self.current_map[x][y][z] == self.final_structure[x][y][z] == map_data.GridStatus.NOT_WALKABLE.value and
                            self.current_map[x][y][z + 1] == map_data.GridStatus.WALKABLE.value):
                            if [x, y, z] not in self.priority_snapshot:
                                struct.append((x, y, z + 1))
        
        def iw_location_distance(coord):
            ix, iy, iz = coord
            return abs(ix - self.leading_foot_loc[0]) ** 2 + (iy - self.leading_foot_loc[1]) ** 2
            
        sorted_struct = sorted(struct, key=iw_location_distance)
        if sorted_struct:
            # print(f"new struct position: {sorted_struct[0]}")
            return sorted_struct[0]
        else:
            return self.leading_foot_loc

    # ---------------------------- IW block communication functions ----------------------------- 

    def send_block_location(self):
        """
        IW does the UART communication to send the block location 
        """

        buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

        # block_change is the data that needs to be sent
        block_change = struct.pack('B', self.id) # indicate that an inchworm is sending this message

        # TODO replace this with self.goal
        for c in self.next_block_loc:
            block_change += struct.pack('B', c)

        block_change += struct.pack('B', map_data.GridStatus.NOT_WALKABLE.value) + struct.pack('B', self.IW_message_counter)
        self.IW_message_counter += 1

        # calculate message length and checksum
        msg_len = (len(block_change) + 2).to_bytes(2,'little') # account for the checksum and end byte
        checksum = self.crc16(block_change).to_bytes(2, 'little')

        # append msg_len, block_change, checksum, ending_code(enum) to buffer
        buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.Initialization.value)

        print(Fore.CYAN + f"Block location buffer {buffer}")
        self.IW_SERIAL.write(buffer)

        sleep(COMMUNICATION_TIMER)
        print(Fore.CYAN + f"block data sent!!")

    def send_block_being_placed(self):
        """
        IW sends the Hex code back to the block indicating that it's being placed
        """
        buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

        # block_change is the data that needs to be sent
        block_change = struct.pack('B', self.id) # indicate that an inchworm is sending this message

        block_change += struct.pack('B', self.IW_message_counter)
        self.IW_message_counter += 1

        msg_len = (len(block_change)+2).to_bytes(2,'little')
        checksum = self.crc16(block_change).to_bytes(2, 'little')

        buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.BeingPlaced.value)
        print(Fore.CYAN + f"Being placed buffer: {buffer}")

        self.IW_SERIAL.write(buffer)
        sleep(COMMUNICATION_TIMER)

        print(Fore.CYAN + f"Indicated block is in placing status!!")
        # TODO: handle transmission error

    def received_block_confirmation(self): 
        print(Fore.CYAN + f"We're trying to confirm the block's existence & ability to communicate, but we haven't been implemented yet D:")
        return True
    
    def send_IW_path_to_block(self, clear_path_com, iw_path):
        """
        Sends the IW path to the structure one grid at a time 

        Args: 
            clear_path_com [list[list]]: the path of the inchworm to be removed from the structure
            iw_path [list[list]]: the path of the inchworm 
        """
        # TODO: IW_path is in X, Y, Z format!!
        # iterate through the iw_path

        # TODO: replace this
        # print(Fore.CYAN + "length of clear IW path", len(clear_path_com))
        for grid_cell in clear_path_com:
            buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

            # block_change is the data that needs to be sent
            block_change = struct.pack('B', self.id) # indicate that an inchworm is sending this message
            
            for c in grid_cell:
                block_change += struct.pack('B', c)

            reverted_id = map_data.revert_status(self.current_map,grid_cell[0], grid_cell[1], grid_cell[2])
            block_change += struct.pack('B', reverted_id) + struct.pack('B', self.IW_message_counter)
            self.IW_message_counter += 1

            msg_len = (len(block_change)+2).to_bytes(2,'little')
            checksum = Inchworm.crc16(block_change).to_bytes(2, 'little')

            # append msg_len, block_change, checksum, ending_code(enum) to buffer

            buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.Changes.value)
            print(Fore.CYAN + f"Clear Path Buffer {buffer}")

            self.IW_SERIAL.write(buffer)

            # delay to make sure all the data is transmitted 
            sleep(COMMUNICATION_TIMER)

        sleep(COMMUNICATION_TIMER)
        print(Fore.CYAN + f"-------------------- actually send the path --------------------")
        # print(Fore.CYAN + f"length of path: {len(iw_path)}")

        for grid_cell in iw_path:
            buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

            # block_change is the data that needs to be sent
            block_change = struct.pack('B', IW_identifier) # indicate that an inchworm is sending this message

            for c in grid_cell:
                block_change += struct.pack('B', c)

            block_change += struct.pack('B', self.iw_path_id) + struct.pack('B', self.IW_message_counter)
            self.IW_message_counter += 1

            msg_len = (len(block_change)+2).to_bytes(2,'little')
            checksum = Inchworm.crc16(block_change).to_bytes(2, 'little')

            # append msg_len, block_change, checksum, ending_code(enum) to buffer

            buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.Changes.value)
            print(Fore.CYAN + f"IW Path Buffer: {buffer}")

            self.IW_SERIAL.write(buffer)

            # delay to make sure all the data is transmitted 
            sleep(COMMUNICATION_TIMER)

    def inchworm_gets_map(self):
        print(Fore.CYAN + f"Getting the map RAHHHHHHHHHHH")

        # Receiving Map Snapshot from Structure
        buffer = []
        msgLenBytes = []
        msgLen = 0
        msgLenCollected = True
        msgLenReceivedCounter = 0
        bytesRead = 0
        collecting_data = False
        while True:
            # print(Fore.CYAN + self.IW_SERIAL.read(1))
            byte = self.IW_SERIAL.read(1)           #read serial port

            if byte == bytearray(struct.pack('B', UART_CODES.StartByte.value)): # and not collecting_data:  # Start byte detected
                buffer = []  
                bytesRead = 0
                msgLenCollected = False
                msgLenBytes = []
                msgLenReceivedCounter = 0
                
            elif not msgLenCollected and msgLenReceivedCounter < 2: # Collecting Message Length
                msgLenBytes.append(byte[0])
                msgLenReceivedCounter += 1
                if msgLenReceivedCounter == 2:
                    # Convert collected bytes to integer (assuming big-endian format)
                    msgLen = int.from_bytes(bytes(msgLenBytes), 'little')
                    msgLenCollected = True
                    collecting_data = True

            elif byte == bytearray(struct.pack('B', UART_CODES.MapSnapshot.value)) and  bytesRead >= msgLen: # Receiving Map Snapshot from Structure
                if collecting_data:
                    buffer = b''.join(buffer) # convert to bytes object
                    checksum = Inchworm.get_checksum(buffer)
                    calculated_check_sum = []
                    calculated_check_sum += Inchworm.crc16(buffer[:-2]).to_bytes(2, 'little')
                    calculated_check_sum = int.from_bytes(bytes(calculated_check_sum), 'big')

                    if checksum == calculated_check_sum:
                        self.current_map = Inchworm.process_received_map_snapshot(buffer)
                        return True
                    else:
                        print(Fore.CYAN + "CHECKSUM DID NOT MATCH")
                        self.state = IW_STATE.ERROR
                        return False

                collecting_data = False
            
            elif collecting_data:
                buffer.append(byte) # Append bytes to buffer if between start and end delimiters
                bytesRead += 1
    
    @staticmethod
    def process_received_map_snapshot(map_data):
        print(Fore.CYAN + f"Processing map data")
        # TODO: CHANGE THISSS PLSSS make generic instead of using JUST NUMBERS
        layers, rows, cols = 8, 8, 4
        array = [[[0 for _ in range(cols)] for _ in range(rows)] for _ in range(layers)]
        index = 0
        for l in range(layers):
            for r in range(rows):
                for c in range(cols):
                    if index < len(map_data):
                        array[l][r][c] = map_data[index]
                        index += 1
        # print(Fore.CYAN + f"Received 3D Array: {array}")
        return array  
        

    def request_map_snapshot(self):
        print(Fore.CYAN + f"Gimme map plsss")
        """
        IW sends the Hex code to the block requesting the map
        """
        buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

        # block_change is the data that needs to be sent
        block_change = struct.pack('B', self.id) # indicate that an inchworm is sending this message

        msg_len = len(block_change).to_bytes(2,'little')

        buffer += msg_len + block_change + struct.pack('B', UART_CODES.NewInchworm.value)

        print(Fore.CYAN + f"Request Map Buffer: {buffer}")
        self.IW_SERIAL.write(buffer)
    
    # Checksum protocol for the IW and Block communication
    @staticmethod
    def get_checksum(buffer): # Get checksum from buffer
        checksum =[]
        checksum += buffer[-2:]

        checksum = int.from_bytes(bytes(checksum), 'big')
        return checksum
    
    # Checksum protocol for the IW and Block communication
    @staticmethod
    def crc16(data: bytes, poly=0x8408):
        '''
        CRC-16-CCITT Algorithm
        '''
        data = bytearray(data)
        crc = 0xFFFF
        for b in data:
            cur_byte = 0xFF & b
            for _ in range(0, 8):
                if (crc & 0x0001) ^ (cur_byte & 0x0001):
                    crc = (crc >> 1) ^ poly
                else:
                    crc >>= 1
                cur_byte >>= 1
        crc = (~crc & 0xFFFF)
        crc = (crc << 8) | ((crc >> 8) & 0xFF)

        return crc & 0xFFFF
    
    ### STATE MACHINE -------------------------------------------------------------------------------------------------------------------------

    def run(self):
        while self.state != IW_STATE.STRUCTURE_COMPLETE:
            self.update_state()

    def update_state(self):
        match self.state:
            case IW_STATE.IDLE:
                self.handle_idle()
            case IW_STATE.INITIALIZATION:
                self.handle_initilization()
                if self.iw_reached_seed_block_flag: # Did IW reach the seed block flag
                    if self.IW_gets_Map_Snapshot(): # IW got the mapsnap shot 
                        if self.is_stuck_or_blocking():
                            self.handle_IW_blocked()
                        else:
                            self.handle_IW_gets_Map()
            case IW_STATE.PATH_PLANNING:
                if self.is_Path_Available(): # Path exists!
                    self.path_exists()
                elif self.no_blocks_left():
                    self.handle_no_blocks_to_place()
                elif self.is_stuck_or_blocking():
                    self.handle_IW_blocked()
                else: # Path doesn't exist!
                    print(Fore.MAGENTA + f" IW{self.id}: Retrying path planning after waiting")
                    self.retry_path() 
            case IW_STATE.TRAVELLING_TO_SUPPLY:
                if self.is_IW_in_supply():
                    self.handle_at_supply()
            case IW_STATE.TRANSPORTING_BLOCK:
                if self.is_IW_in_block():
                    self.handle_transported_block()
            case IW_STATE.PLACING_BLOCK:
                if self.incorrect_block_location(): # block is placed in the wrong location
                    self.handle_error()
                elif self.IW_gets_Map_Snapshot(): # assume that the block is placed in the correct location
                    self.IW_clear_path()
                    if self.is_structure_complete(self.current_map, self.final_structure): # structure is complete
                        self.handle_structure_complete()
                    else: # structure is incomplete
                        self.handle_structure_incomplete()
            case IW_STATE.ERROR:
                self.handle_error()
            case IW_STATE.STRUCTURE_COMPLETE:
                self.handle_structure_complete()
            case IW_STATE.YIELDING:
                self.handle_yielding()
                
    def set_state(self, new_state):
        if new_state != self.state:
            self.prev_state = self.state
            self.state = new_state
            print(Fore.BLUE + f"IW{self.id}: State changed from {self.prev_state.name} to {self.state.name}")
        else: 
            print(Fore.BLUE + f"IW{self.id} *tried* to change from {self.state.name} to the inputted {new_state.name}")
    
    ##### Checkers and Handlers

    # Handlers 

    # added this func incase we need it in the future
    def handle_idle(self):
        if self.print_flag:
            # print(Fore.BLUE + "IDLINGGG....")
            self.print_flag = False
    
    # during the initiliaztion phase the inchworm should lift up it's gripper and touch the seed block
    # and transfer the block location to the seed block
    def handle_initilization(self):
        print(Fore.BLUE + f"IW{self.id}: MOVINGGG TO SEED BLOCK: press n to step")
        if self.paths: # this happens second 
            # move IW in sim
            if MANUAL_TESTING:
                self.dummy_IW_move()
            if self.leading_foot_loc == self.goal: 
                print(Fore.BLUE + " Touching the seed block")
                self.iw_reached_seed_block_flag = True

        else: # this happens first 
            # Find & path plan to seed block 
            self.plan_path(SEED_BK)     
        
    def handle_IW_gets_Map(self):
        print(Fore.BLUE + f"IW{self.id}: Map snapshot successful. Now path planning...")
        self.IW_clear_path()
        self.plan_path()
        self.set_state(IW_STATE.PATH_PLANNING)

    def path_exists(self):
        # MOOOO HELPPP 
        print(Fore.BLUE + f"IW{self.id}: Path found. Sending the IW path to the structure")
        # IW sends it's path to the structure 
        if not SIMULATION:
            self.send_IW_path_to_block(self.clear_path_com, self.paths)
        # TODO !!!!! 
        print(Fore.BLUE + f" IW{self.id}: Travelling to the supply")
        self.set_state(IW_STATE.TRAVELLING_TO_SUPPLY)

    def handle_IW_blocked(self):
        print(Fore.BLUE + f"IW{self.id}: YIELDING!!! GOTTA GET OUTTA DA WAY")
        self.temp_path = [self.lagging_foot_loc, self.leading_foot_loc]
        if not SIMULATION:
            # If no path is available, set the path as the inchworm's location, so other IWs still know to avoid it
            self.paths = self.temp_path
            self.send_IW_path_to_block(self.clear_path_com, self.paths)
            sleep(PATH_PLANNING_TIMER) # TODO: Decide if we need a  sleep here because we want to have a non blocking code
        px, py, pz = self.find_nearest_structure()
        self.goal = [px, py, pz]
        
        if self.goal != self.leading_foot_loc:
            self.plan_path(bypass_flag=True)
            self.set_state(IW_STATE.YIELDING)
            
        if SIMULATION and self.paths == []:
            self.paths = self.temp_path
            self.set_state(IW_STATE.YIELDING)
    
    def retry_path(self):
        # Question: Is it ok for the IW to sleep?!! cuz then it doesn't get active data yk 
        self.temp_path = [self.lagging_foot_loc, self.leading_foot_loc]
        if not SIMULATION:
            # If no path is available, set the path as the inchworm's location, so other IWs still know to avoid it
            self.paths = self.temp_path
            self.send_IW_path_to_block(self.clear_path_com, self.paths)
            sleep(PATH_PLANNING_TIMER) # TODO: Decide if we need a  sleep here because we want to have a non blocking code
        # or stay here until the IW gets a new map!!
        # MOOO HELPPP
        self.plan_path()
        # For simulation. If no path is available, save the path as its own location 
        if SIMULATION and self.paths == []:
            self.paths = self.temp_path
        # print(self.current_map)

    def handle_no_blocks_to_place(self): 
        self.set_state(IW_STATE.STRUCTURE_COMPLETE)

    def handle_at_supply(self):
        print(Fore.BLUE + f"IW{self.id}: Touching the new block (move)")
        # touch the new block
        if not SIMULATION: 
            print(Fore.BLUE + f" IW{self.id}: IW flashes block with its location")
            self.send_block_location()
            # pause so that the block has enough time to process the info
        
        self.set_state(IW_STATE.TRANSPORTING_BLOCK)

    def handle_transported_block(self):
        if not SIMULATION:
            if self.received_block_confirmation():
                print(Fore.BLUE + f"IW{self.id}: IW sends a messgae indicating block is being placed")
                # IW sends a messgae indicating block is being placed
                # MOOOOO HELPPPP
                self.send_block_being_placed()
            else: 
                self.handle_error()
        self.set_state(IW_STATE.PLACING_BLOCK)

    def IW_clear_path(self):
        self.current_map = map_data.rm_inchworm_path_from_grid(self.current_map, self.paths, self.id)

        self.clear_path_com = copy.deepcopy(self.paths)
        self.paths = [] # Reset current path 
        self.step_instructions = []
        self.step_num = 1
        self.holding_block = False
        self.goal = self.leading_foot_loc

        print(Fore.RED + f"IW{self.id}: Reset the path")
        
        # self.set_state(IW_STATE.PLACING_BLOCK)

    def handle_error(self):
        print(Fore.BLUE + f"IW{self.id}: OHHH NOOO, ERROR ERROR")
        print(Fore.BLUE + f" IW{self.id}: Stop Inchworm") # stop the inchworm
        print(Fore.RED + f" IW{self.id}: flash red LED") # flash red Led 

        self.set_state(IW_STATE.IDLE)
    
    def handle_structure_complete(self):
        print(Fore.BLUE + f"IW{self.id}: Structure is complete YIppeee")
        print(Fore.BLUE + f" IW{self.id}: IW Stopped") # stop the iW
        print(Fore.GREEN + f" IW{self.id}: flash Green LED") # flash green light

        self.set_state(IW_STATE.STRUCTURE_COMPLETE)

        if not SIMULATION:
            self.get_logger().info("Operation complete. Shutting down ROS.")
            rclpy.shutdown()

    def handle_structure_incomplete(self):
        print(Fore.BLUE + f"IW{self.id}: Structure is incomplete. Updated IW's map with placed block. Finding new path...")
        self.plan_path()
        self.set_state(IW_STATE.PATH_PLANNING)
        
    def handle_yielding(self):
        """ step out of the way if inchworm is in another's path """
        if not self.is_Path_Available():
            print(Fore.BLUE + f"IW{self.id}: stuck and cannot get out of way, waiting for updates")
            sleep(PATH_PLANNING_TIMER)
        else:
            # print("we moved and we chillll")
            self.plan_path()
        self.set_state(IW_STATE.PATH_PLANNING)

    # Checkers
    def IW_gets_Map_Snapshot(self):
        # blah blah low level language 
        # TODO: ask Mo for help when the IW gets the map SnapShot back 
        # return true if the IW got the map snapshot
        if SIMULATION: 
            if self.leading_foot_loc == self.goal: 
                print(Fore.BLUE + f"IW{self.id}: IW got map snapshot")
                return True
            return False
        else:                
            # request the map
            if self.state == IW_STATE.INITIALIZATION:
                self.request_map_snapshot()
            return self.inchworm_gets_map()
    
    def is_Path_Available(self):
        # print(Fore.BLUE + f"IW{self.id}: Checking path availability... {self.paths}")
        if self.paths and self.goal and self.paths != self.temp_path:
            return True
        else: 
            return False

    def no_blocks_left(self):
        return (self.goal == [-1, -1, -1])

    def is_IW_in_supply(self):
        """ return true if the IW is in the supply location (check the flag and compare the current IW  location through dead reckoning and the supply location)"""
        # print(Fore.BLUE + f"IW{self.id}: Checking if at supply location...")
        # print(Fore.BLUE + f"IW{self.id}: If in sim, press n to step")
        if not SIMULATION:
            if MANUAL_TESTING: 
                self.dummy_IW_move()
            # if INCHWORM_MOVED and MANUAL_TESTING:
            #     IW_in_supply = input("Is iW in supply location? (yes/no) \n")
            #     if IW_in_supply.lower() == 'yes':
            #         self.get_next_step()
        # TODO: Replace with actual implementation
        for bd_loc in BD_LOCS:
            if [bd_loc[0], bd_loc[1], bd_loc[2]-1] == self.leading_foot_loc: 
                print(Fore.BLUE + f"IW{self.id}: IW thinks it's at the supply depot")
                # self.holding_block = True
                return True 
        return False

    def is_IW_in_block(self):
        """return true if the IW is in the block location (check the flag and compare the current IW  location through dead reckoning and the block location)"""
        print(Fore.BLUE + f"IW{self.id}: Checking if at block location...")
        # print(Fore.BLUE + f"IW{self.id}: If in sim, press n to step")
        # TODO: Replace with actual implementation
        if not SIMULATION:
            if MANUAL_TESTING: 
                self.dummy_IW_move()
            # if INCHWORM_MOVED and MANUAL_TESTING:
            #     IW_in_supply = input("Is iW in block location? (yes/no) \n")
            #     if IW_in_supply.lower() == 'yes':
            #         self.get_next_step() 

        if self.leading_foot_loc == self.goal: 
            print(Fore.BLUE + f" IW{self.id}: IW thinks it's at the incoming block loc")
            # self.holding_block = False
            return True 
        else: 
            return False

    def incorrect_block_location(self):
        """ return true if the IW gets "incorrectly placed block" from the structure """
        # MOOOO HELLPOPPPP
        if SIMULATION: 
            return not self.leading_foot_loc == self.goal 
        else: 
            pass
        
    def is_stuck_or_blocking(self):
        """ return true if the IW is stuck at the start or is currently in another iw's path"""
        # if self.yield_cooldown_timer > 0:
        #     print("nuh uh uh cannot yield yet")
        #     return False # cool down bucko
        
        def other_iw_at(loc):
            try:
                status = self.current_map[loc[0]][loc[1]][loc[2]]
                if map_data.GridStatus.is_inchworm_path(status):
                    iw_id = map_data.GridStatus.which_inchworm(status)
                    if iw_id is not None and iw_id != self.id:
                        return iw_id
            except IndexError:
                pass
            return None
        
        lead_blocker = other_iw_at(self.leading_foot_loc)
        lag_blocker = other_iw_at(self.lagging_foot_loc)
        # print(f"lead: {lead_blocker} and lag: {lag_blocker}")
        
        if lead_blocker or lag_blocker:
            if lag_blocker:
                other_iw = lag_blocker
            else:
                other_iw = lead_blocker
            # print(Fore.YELLOW + f"IW{self.id} is blocking or blocked by IW{other_iw}!")
            return True
        # print("no need for yields")
        return False
    
    def is_structure_complete(self, curr_map, final_map):
        map_complete = map_data.is_structure_complete(curr_map, final_map)
        return map_complete
    
## ROS 2 FUNCTIONALITY -------------------------------------------------------------------

if not SIMULATION:
    class InchwormNode(Node): 
        def __init__(self): 
            # Initialize the ROS node the inchworm uses 
            super().__init__('inchworm_node')
            # Initialize the inchworm (state machine)
            with open("/home/smac/robot_ws/src/SMAC6.0/Final_Structure.json", "r") as final_map_file:
                final_structure = json.load(final_map_file)
            self.inchworm = Inchworm(IW_1_ORIENTATION, final_structure, IW_1_LOC)

            # Every second, update the state
            self.create_timer(1, self.update_state)

            # Set up the the inchworm node (state machine) as the client for the action of stepping
            self._action_client = ActionClient(self, Inchwormpath, 'inchworm_moving')
            self.get_logger().info("Inchworm Node Initialized")
        
        def update_state(self): 
            """Update the inchworm state machine & send an existing set of step instructions to the motors """
            self.inchworm.update_state()

            if self.inchworm.step_instructions != []: 
                self.send_goal(self.inchworm.step_instructions)
                # Clear the step instructions so that the state continues updating, 
                # but the instructions are not resent 
                self.inchworm.step_instructions = []

        def send_goal(self, all_steps):
            """Send an action request for the 'inchworm_moving' action"""
            # Initialize action goal message 
            goal_msg = Inchwormpath.Goal()
            goal_msg.all_steps = []

            # Since steps contain complex data, split the step instructions 
            for step_type, step_change in all_steps: 
                # Create a Step msg to be put in a list
                step_msg = Step(step_type=step_type, step_change=step_change)
                goal_msg.all_steps.append(step_msg)

            # Return error if the action server times out 
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Action server (execute_path.py) not available")
                return

            # Asynchronously send goal msg. While it's running, receive feedback in this node through feedback_callback
            self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            # Upon successful receival of the action command, run the goal_response_callback
            self._send_goal_future.add_done_callback(self.goal_response_callback)

        def goal_response_callback(self, future):
            """Runs upon successful receival of the goal command"""
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
                return

            self.get_logger().info('Goal accepted :)')

            # Asynchronously receive the result of the action
            self._get_result_future = goal_handle.get_result_async()
            # Upon successful completion of the action command, run the goal_result_callback
            self._get_result_future.add_done_callback(self.get_result_callback)

        def get_result_callback(self, future):
            """Runs upon successful completion of the action"""
            result = future.result().result
            self.get_logger().info(f'Result: Completed? {result.completion_status}')
            # rclpy.shutdown()

        def feedback_callback(self, feedback_msg):
            """Runs repeeatedly as the action runs, providing feedback"""
            feedback = feedback_msg.feedback
            self.get_logger().info(f'Feedback: Step {feedback.step_num} / {feedback.total_steps}')

def main(args=None):
    if not SIMULATION:
        rclpy.init(args=args)
        inchworm_node = InchwormNode()
        rclpy.spin(inchworm_node)
    elif MANUAL_TESTING:
        with open("/home/smac/robot_ws/src/SMAC6.0/Final_Structure.json", "r") as final_map_file:
            final_structure = json.load(final_map_file)
        inchworm = Inchworm(orientation=IW_ORIENTATIONS[0], final_structure=final_structure, location=IW_LOCS[0], holding_block=False)
        try:
            inchworm.run()
        except KeyboardInterrupt:
            print(Fore.GREEN + "Stopping the inchworm system.") 
        inchworm_node.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()
   