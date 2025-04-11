from enum import Enum
import copy
from config import *
import map_data
from blueprint import blueprint
from time import sleep
import serial
import struct
import json
from colorama import Fore, init
import numpy as np
init(autoreset=True)

###### UART stuff
UART_BAUD = 9600 # config

# Pins 
# GPIO 15, pin 8 = RX yellow wire 
# GPIO 14, pin 10 = TX green wire
# ground = Pin 14
# test 
DEBUG = True # print statements to help DEBUG 
INCHWORM_MOVED = False # setting this to True so it can bypass all the movements for debugging 

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

PATH_PLANNING_TIMER = 5
COMMUNICATION_TIMER = 0.5 # DO NOT CHANGE THIS ANY LESS THAN 0.4

lagging_transform = {
    InchwormOrientation.NORTH: lambda x, y, z: (x, y - 1, z),  
    InchwormOrientation.SOUTH: lambda x, y, z: (x, y + 1, z),
    InchwormOrientation.EAST: lambda x, y, z: (x - 1, y, z), 
    InchwormOrientation.WEST: lambda x, y, z: (x + 1, y, z) 
}

class Inchworm:
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
        self.goal = [] # goal coord
        self.goal_progress_index = 0
        self.num_steps = 0
        self.step_num = 1
        # self.found_structures = []
        # self.misc_blocks = []
        self.iw_path_id = map_data.GridStatus.inchworm_path(self.id) 
        self.clear_path_com = [] # stores the list of path to send to the blocks to clear from the map 
        self.IW_message_counter = 1 # this is the messgae counter for sending data, IW's message counter increases

        # Leg locations for the inchworm. 
        self.leading_foot_loc = location
        self.lagging_foot_loc = list(lagging_transform[orientation](*self.leading_foot_loc))
        
        # pertaining to the state machine 
        self.state = IW_STATE.INITIALIZATION
        self.print_flag = True
        self.intilization_path_flag = False

        
        Inchworm.next_id += 1
        Inchworm.inchworm_list.append(self)

        # UART stuff
        if not SIMULATION: 
            self.IW_SERIAL = serial.Serial ("/dev/ttyAMA0", 9600)    #Open port with baud rate
        else: 
            # These are vars that the simulation uses to simulate each of the inchworm feet
            self.last_cell, self.last_bk_og_texture, self.last_cell_2, self.last_bk_og_texture_2, self.spawned, self.prev_held_block_loc = None, None, None, None, False, [0,0,0]

    def __del__(self):
        """
        Deletion of inchworm in the list of inchworms.
        """
        Inchworm.inchworm_list = [iw for iw in Inchworm.inchworm_list if iw.id != self.id]
    
    def dummy_IW_move(self):
        make_IW_move = input("Make IW move? (yes/no) \n")
        if make_IW_move.lower() == 'yes':
            INCHWORM_MOVED = True
        elif make_IW_move.lower() == 'no':
            INCHWORM_MOVED = False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")

    def plan_path(self, next_goal: tuple[int, int, int] = None): 
        """ Plan path from current location to specified goal. """
        # print(Fore.MAGENTA + f"IW{self.id}, leading: {self.leading_foot_loc}, lagging foot loc: {self.lagging_foot_loc}")

        is_traveling = False # assumes that if not specified, objective is to travel, not place
        if next_goal == None:
            print(Fore.MAGENTA + f"IW{self.id}: goal not given... finding goal now")
            # print(Fore.MAGENTA + "(PP) current_map: ", self.current_map)
            # print(Fore.MAGENTA + "(PP) final_map: ", self.final_structure)
            self.goal = blueprint(self.current_map, self.final_structure) # gets goal from blueprint if none is given
            if self.goal == [-1, -1, -1]: # structure is complete!!
                print(Fore.MAGENTA + f"IW{self.id}: Structure is complete")
                return
            elif self.goal == [-9, -9, -9]:
                print(Fore.MAGENTA + f"IW{self.id}: Erm... No goal was given... No structure was found...")
                path = []
                return
            if [self.goal[0], self.goal[1], self.goal[2]+1] != SEED_BK:
                # print(Fore.MAGENTA + f"IW{self.id}: Setting IW's goal to be incoming block")
                self.current_map = map_data.update_grid_status(self.current_map, self.goal, map_data.GridStatus.INCOMING_BLOCK.value) # updates map for next_goal to be incoming_block
        else:
            is_traveling = True
            self.goal = next_goal
            if self.goal != SEED_BK: 
                print(Fore.MAGENTA + f"IW{self.id}: Goal is not seed block. Setting IW's goal to be incoming block")
                self.current_map = map_data.update_grid_status(self.current_map, self.goal, map_data.GridStatus.INCOMING_BLOCK.value) # updates map for next_goal to be incoming_block

        try: 
            step_instructions, steps, path = [], [], []
            if is_traveling or self.holding_block:
                # Find one path, to travel to the specified goal
                path, steps, new_orientation = map_data.initiate_find_path(self.current_map, self.lagging_foot_loc, self.goal, self.orientation, self.holding_block, self.id)
            else:
                # Find path to block depot 
                bd_path, bd_steps, new_orientation = map_data.initiate_find_path(self.current_map, self.lagging_foot_loc, BD_1_LOC, self.orientation, self.holding_block, self.id)
                self.holding_block = True

                # If it doesn't find a path to the supply depot, just return, don't bother trying to path plan further
                if bd_path == []: 
                    return
                
                # Find path to where the next block will be placed
                goal_path, goal_steps, new_orientation = map_data.initiate_find_path(self.current_map, bd_path[-2], self.goal, new_orientation, self.holding_block, self.id)
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
            step_getter(step_instructions)
            if SIMULATION: # Avoid unnecessary data usage by only saving step instructions twice in simulation. 
                # Saving the step instructions like this enables the step instructions to be stored for *each* simulated inchworm, rather than just one at a time 
                # (Storing the step instructions to a separate file is a limited to just one IW if running simulation.) 
                # TODO: determine if storing to steps.txt is really necessary? 
                self.step_instructions = step_instructions
                print(Fore.BLUE + f"IW{self.id}: step instructions: {self.step_instructions}")
        except RuntimeError as e:
            print(Fore.MAGENTA + f"IW{self.id}: Error: {e}. No path found, try again later.")
            return

    def get_next_point(self): 
        """ 
        Returns the set of the next points of inchworm travel. Used for stepping through path for sim.
        """
        if self.goal_progress_index > 0:
            self.leading_foot_loc = self.paths[self.goal_progress_index]  # Get the next point
            self.lagging_foot_loc = list(lagging_transform[self.orientation](*self.leading_foot_loc))

            # self.lagging_foot_loc = self.paths[self.goal_progress_index - 1]
        
        x, y, z = self.leading_foot_loc
        self.goal_progress_index += 1
        
        if ([x, y, z] == [BD_1_LOC[0], BD_1_LOC[1], BD_1_LOC[2]-1]):
            self.holding_block = True
        elif self.holding_block & ([x, y, z] == [self.goal[0], self.goal[1], self.goal[2]-1]):
            self.holding_block = False
        
        if self.holding_block and [x, y, z] != self.goal:
            z = z + 1
        return x, y, z
    
    def get_next_step(self):
        """Returns the leading foot location as is used for the simulation"""
        if self.step_num > self.num_steps: 
            ValueError(Fore.BLUE + f"Erm we're on step {self.step_num} but there should be {self.num_steps} steps")
        else: 
            if self.goal_progress_index > 0:
                if SIMULATION: 
                    step_str = self.step_instructions[self.step_num-1]
                else: 
                    file = open('steps.txt') 
                    content = file.readlines() 
                    step_str = content[self.step_num-1]
                print(Fore.BLUE + f"IW{self.id}: Next step: {step_str}. This is step {self.step_num}/{self.num_steps} for path of length {len(self.paths)}")

                # Update Inchworm Orientation with each step
                self.orientation = map_data.get_orientation(step_str, self.orientation)

                self.leading_foot_loc = self.paths[self.goal_progress_index]  # Get the next point # step_num
                if "PLACE" not in step_str:
                    self.lagging_foot_loc = list(lagging_transform[self.orientation](*self.leading_foot_loc))
                    if "UP" in step_str: 
                        self.lagging_foot_loc[2] = self.leading_foot_loc[2] - 1
                self.step_num += 1
       
            x, y, z = self.leading_foot_loc
            self.goal_progress_index += 1
            
            if ([x, y, z] == [BD_1_LOC[0], BD_1_LOC[1], BD_1_LOC[2]-1]):
                self.holding_block = True
            elif self.holding_block & ([x, y, z] == [self.goal[0], self.goal[1], self.goal[2]-1]):
                self.holding_block = False
            
            if self.holding_block and [x, y, z] != self.goal:
                z = z + 1
            print(f"IW{self.id}: foot locs: {[x, y, z]}, {self.lagging_foot_loc}")
            return x, y, z
    
    def get_total_inchworms(cls):
        """
        Returns the total number of inchworms in the system
        """
        return len(cls.inchworm_list)
        
    def reset_inchworms(cls):
        cls.next_id = 1
        cls.inchworm_list.clear()


    # ---------------------------- IW block communication functions ----------------------------- 

    def send_block_location(self):
        """
        IW does the UART communication to send the block location 
        """

        buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

        # block_change is the data that needs to be sent
        block_change = struct.pack('B', self.id) # indicate that an inchworm is sending this message

        print ("Incoming block location", self.goal)

        # TODO replace this with self.goal
        for c in self.goal:
            block_change += struct.pack('B', c)

        block_change += struct.pack('B', map_data.GridStatus.NOT_WALKABLE.value) + struct.pack('B', self.IW_message_counter)
        self.IW_message_counter += 1

        print(Fore.RED + "Block change", block_change)

        # calculate message length and checksum

        

        msg_len = (len(block_change) + 2).to_bytes(2,'little') # account for the checksum and end byte
        checksum = self.crc16(block_change).to_bytes(2, 'little')


        # append msg_len, block_change, checksum, ending_code(enum) to buffer

        buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.Initialization.value)

        print("Block location buffer", buffer)
        self.IW_SERIAL.write(buffer)

        sleep(COMMUNICATION_TIMER)
        print(Fore.RED + "block data sent!!")
        # TODO: handle transmission error

    def send_block_being_placed(self):
        """
        IW sends the Hex code back to the block indicating that it's being placed
        """
        buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

        # block_change is the data that needs to be sent
        block_change = struct.pack('B', self.id) # indicate that an inchworm is sending this message

        # for c in map_data.GridStatus.INCOMING_BLOCK.value:
        #     block_change += struct.pack('B', c)

        block_change += struct.pack('B', map_data.GridStatus.INCOMING_BLOCK.value) + struct.pack('B', self.IW_message_counter)
        self.IW_message_counter += 1

        # print(Fore.RED + "Block change", block_change)

        # calculate message length and checksum

        msg_len = (len(block_change)+2).to_bytes(2,'little')
        checksum = self.crc16(block_change).to_bytes(2, 'little')

        # print(Fore.RED + "msg_len", msg_len)
        # print(Fore.RED + "checksum", checksum)

        # append msg_len, block_change, checksum, ending_code(enum) to buffer

        buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.BeingPlaced.value)
        print("Being placed buffer: ", buffer)

        self.IW_SERIAL.write(buffer)
        sleep(COMMUNICATION_TIMER)

        print(Fore.RED + "Indicated block is in placing status!!")
        # TODO: handle transmission error

    def received_block_confirmation(self): 
        print(Fore.RED + "We're trying to confirm the block's existence & ability to communicate, but we haven't been implemented yet D:")
        return True
    
    def send_IW_path_to_block(self, clear_path_com, iw_path):
        """
        Sends the IW path to the structure one grid at a time 

        Args: 
            iw_path [list[list]]: the path of the inchworm 
        """
        # TODO: IW_path is in X, Y, Z format!!
        # iterate through the iw_path

        # TODO: replace this
        print("length of clear IW path", len(clear_path_com))
        for grid_cell in clear_path_com:
            if DEBUG:
                print("Grid Cell path", grid_cell)
            buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

            # block_change is the data that needs to be sent
            block_change = struct.pack('B', IW_identifier) # indicate that an inchworm is sending this message
            
            for c in grid_cell:
                block_change += struct.pack('B', c)

            reverted_id = map_data.revert_status(self.current_map,grid_cell[0], grid_cell[1], grid_cell[2])
            block_change += struct.pack('B', reverted_id) + struct.pack('B', self.IW_message_counter)
            self.IW_message_counter += 1


            msg_len = (len(block_change)+2).to_bytes(2,'little')
            checksum = Inchworm.crc16(block_change).to_bytes(2, 'little')

            # append msg_len, block_change, checksum, ending_code(enum) to buffer

            buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.Changes.value)
            print("buffer", buffer)

            self.IW_SERIAL.write(buffer)

            # delay to make sure all the data is transmitted 
            sleep(COMMUNICATION_TIMER)

        sleep(COMMUNICATION_TIMER)
        print("actually send the path --------------------")
        print("length of path: ", len(iw_path))

        for grid_cell in iw_path:
            if DEBUG:
                print("Grid Cell path", grid_cell)
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
            print("buffer", buffer)

            self.IW_SERIAL.write(buffer)

            # delay to make sure all the data is transmitted 
            sleep(COMMUNICATION_TIMER)

        # TODO: handle transmission error


    def inchworm_gets_map(self):
        print("Getting the map RAHHHHHHHHHHH")

        # Receiving Map Snapshot from Structure
        buffer = []
        msgLenBytes = []
        msgLen = 0
        msgLenCollected = True
        msgLenReceivedCounter = 0
        bytesRead = 0
        collecting_data = False
        while True:
            # print(self.IW_SERIAL.read(1))
            byte = self.IW_SERIAL.read(1)           #read serial port
            # if byte == []:
            #     return False
            # print(byte) # b'\xaa'
            # print(ord(byte))
            # byte = ord(byte) # turn it into a decimal value  # 170
            # print(byte)
            # byte = hex(byte) # 0xaa
            # print(byte)

            # byte = bytearray(struct.pack('B', byte))
            # print(byte)
            # print(sys.stdout.buffer.write(bytes(byte)))
            # print(ord(byte))
            # print("Recieved byte", byte)

            # print(bytearray(struct.pack('B', UART_CODES.StartByte.value)))
            # print(byte == bytearray(struct.pack('B', UART_CODES.StartByte.value)))

            if byte == bytearray(struct.pack('B', UART_CODES.StartByte.value)): # and not collecting_data:  # Start byte detected
                # print("start byte detected")
                buffer = []  
                # print("Print bufferrrrr after clear", buffer)
                bytesRead = 0
                msgLenCollected = False
                msgLenBytes = []
                msgLenReceivedCounter = 0
                
            elif not msgLenCollected and msgLenReceivedCounter < 2: # Collecting Message Length
                # print("Collecting Message Length")
                # byte_in_int = ord(byte)
                msgLenBytes.append(byte[0])
                msgLenReceivedCounter += 1
                if msgLenReceivedCounter == 2:
                    # Convert collected bytes to integer (assuming big-endian format)
                    msgLen = int.from_bytes(bytes(msgLenBytes), 'little')
                    # print("msg Len when collecting msg len: ", msgLen)
                    msgLenCollected = True
                    collecting_data = True

            elif byte == bytearray(struct.pack('B', UART_CODES.MapSnapshot.value)) and  bytesRead >= msgLen: # Receiving Map Snapshot from Structure
                # print("BytesRead: ", bytesRead)
                # print("msgLen: ", msgLen)
                if collecting_data:
                    buffer = b''.join(buffer) # convert to bytes object
                    print("BUFFFEERR after join: ", buffer)
                    checksum = Inchworm.get_checksum(buffer)
                    calculated_check_sum = []
                    calculated_check_sum += Inchworm.crc16(buffer[:-2]).to_bytes(2, 'little')
                    calculated_check_sum = int.from_bytes(bytes(calculated_check_sum), 'big')

                    # print("checksum", checksum)
                    # print("calculated_check_sum", calculated_check_sum)

                    # Inchworm.crc16(buffer[:-2])
                    
                    if checksum == calculated_check_sum:
                        self.current_map = Inchworm.process_received_map_snapshot(buffer)
                        # print(self.current_map)
                        return True
                    else:
                        print("CHECKSUM DID NOT MATCH")
                        self.state = IW_STATE.ERROR
                        return False

                collecting_data = False
            
            elif collecting_data:
                buffer.append(byte) # Append bytes to buffer if between start and end delimiters
                bytesRead += 1
    
    @staticmethod
    def process_received_map_snapshot(map_data):
        print("Processing map data")
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
        # print("Received 3D Array:", array)
        return array  
        

    def request_map_snapshot(self):
        print("Gimme map plsss")
        """
        IW sends the Hex code to the block requesting the map
        """
        buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

        # block_change is the data that needs to be sent
        block_change = struct.pack('B', IW_identifier) # indicate that an inchworm is sending this message

        msg_len = len(block_change).to_bytes(2,'little')

        buffer += msg_len + block_change + struct.pack('B', UART_CODES.NewInchworm.value)

        print("Init Buffer: ", buffer)
        self.IW_SERIAL.write(buffer)
        return True 
    
    # Checksum protocol for the IW and Block communication
    @staticmethod
    def get_checksum(buffer): # Get checksum from buffer
        # print("buffer", buffer)
        # checksum = buffer[-2:]
        # print("Checksum: ", checksum)
        # msgLen = int.from_bytes(bytes(msgLenBytes), 'little')
        # checksum =  int.from_bytes(checksum, 'big')  # Convert to integer
        checksum =[]
        checksum += buffer[-2:]

        # checksum.append(buffer.pop())
        # checksum.append(buffer.pop())
        # checksum = bytearray(checksum)
        checksum = int.from_bytes(bytes(checksum), 'big')
        # checksum = int.from_bytes(checksum,'big')
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
                # if self.intilization_path_flag:
                if self.IW_gets_Map_Snapshot(): # IW got the mapsnap shot 
                    self.handle_IW_gets_Map()
            case IW_STATE.PATH_PLANNING:
                if self.is_Path_Available(): # Path exists!
                    self.path_exists()
                elif self.no_blocks_left():
                    self.handle_no_blocks_to_place()
                else: # Path doesn't exist!
                    print(Fore.MAGENTA + f"IW{self.id}: Retrying path planning after waiting")
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

    
    ##### Checkers and Handlers

    # Handlers 

    # added this func incase we need it in the future
    def handle_idle(self):
        if self.print_flag:
            print(Fore.BLUE + "IDLINGGG....")
            self.print_flag = False

    
    # during the initiliaztion phase the inchworm should lift up it's gripper and touch the seed block
    # and transfer the block location to the seed block
    def handle_initilization(self):
        print(Fore.BLUE + f"IW{self.id}: MOVINGGG TO SEED BLOCK: press n to step")
        print("This is my path: ", self.paths)
        print("True?? Paths : ", self.paths == True)
        if self.paths: # this happens second 
            print("I'm hereee")
            # move IW in sim
            if self.goal_progress_index >= len(self.paths) or INCHWORM_MOVED: 
                print(Fore.BLUE + "Touching the seed block")    
                self.current_map = map_data.rm_inchworm_path_from_grid(self.current_map, self.paths)
                print("Clear Path: ", self.clear_path_com)
                self.clear_path_com = self.paths
                self.paths = [] # Reset current path 
                self.goal_progress_index = 0 # TODO: May be good to move to handle_IW_gets_Map or clear_my_path
                self.step_num = 1

                print(Fore.BLUE + "Reset the path.")

        else: # this happens first 
            # Find & path plan to seed block 
            print("Clear Path before path plan to seed blcok: ", self.clear_path_com)
            self.plan_path(SEED_BK)

        
        
    def handle_IW_gets_Map(self):
        print(Fore.BLUE + f"IW{self.id}: Map snapshot successful. Now path planning...")
        if DEBUG:
            print("Current Map from Block")
            print(self.current_map)
        self.clear_path_com = self.paths
        self.plan_path()
        self.state = IW_STATE.PATH_PLANNING
        print(Fore.BLUE + f"Current inchworm state: {self.state}")

    def path_exists(self):
        # MOOOO HELPPP 
        print(Fore.BLUE + f"IW{self.id}: Path found. Sending the IW path to the structure")
        # IW sends it's path to the structure 
        if not SIMULATION:
            self.send_IW_path_to_block(self.clear_path_com, self.paths)
        
        self.state = IW_STATE.TRAVELLING_TO_SUPPLY
        print(Fore.BLUE + f"Current inchworm state: {self.state}")
    
    def retry_path(self):
        # Question: Is it ok for the IW to sleep?!! cuz then it doesn't get active data yk 
        if not SIMULATION:
            sleep(PATH_PLANNING_TIMER) # TODO: Decide if we need a  sleep here because we want to have a non blocking code
        # or stay here until the IW gets a new map!!
        # MOOO HELPPP
        self.clear_path_com = self.paths
        self.plan_path()

    def handle_no_blocks_to_place(self): 
        self.state = IW_STATE.STRUCTURE_COMPLETE
        print(Fore.BLUE + f"Current inchworm state: {self.state}")

    def handle_at_supply(self):
        print(Fore.BLUE + f"IW{self.id}: Touching the new block (move)")
        # touch the new block
        if not SIMULATION: 
            print(Fore.BLUE + f"IW{self.id}: IW flashes block with it's location")
            self.send_block_location()
            # pause so that the block has enough time to process the info

        
        self.state = IW_STATE.TRANSPORTING_BLOCK
        print(Fore.BLUE + f"IW{self.id}: Current inchworm state: {self.state}")

    def handle_transported_block(self):
        if self.received_block_confirmation():
            print(Fore.BLUE + f"IW{self.id}: IW sends a messgae indicating block is being placed")
            # IW sends a messgae indicating block is being placed
            # MOOOOO HELPPPP
            self.send_block_being_placed()
        else: 
            self.handle_error()


        self.state = IW_STATE.PLACING_BLOCK
        print(Fore.BLUE + f"IW{self.id}: Current inchworm state: {self.state}")

    def IW_clear_path(self):
        print("I cleared my pathhhhhh yippeeee")
        print(f"current map {self.current_map}")
        self.current_map = map_data.rm_inchworm_path_from_grid(self.current_map, self.paths, self.id)
        print(f"after clearing {self.current_map}")
        self.clear_path_com = self.paths
        self.paths = [] # Reset current path 
        self.goal_progress_index = 0 # TODO: May be good to move to handle_IW_gets_Map or clear_my_path
        self.step_num = 1
        self.holding_block = False

        print(Fore.BLUE + f"IW{self.id}: Reset the path")

    def handle_error(self):
        print(Fore.BLUE + f"IW{self.id}: OHHH NOOO, ERROR ERROR")

        print(Fore.BLUE + f"IW{self.id}: Stop Inchworm")
        # stop the inchworm
        
        print(Fore.BLUE + f"IW{self.id}: flash red LED")
        # flash red Led 

        self.state = IW_STATE.IDLE
        print(Fore.BLUE + f"IW{self.id}: Current inchworm state: {self.state}")
    
    def handle_structure_complete(self):
        print(Fore.BLUE + f"IW{self.id}: Structure is complete YIppeee")

        # stop the iW
        print(Fore.BLUE + f"IW{self.id}: IW Stopped")
        
        # flash green light
        print(Fore.BLUE + f"IW{self.id}: flash Green LED")

        self.state = IW_STATE.STRUCTURE_COMPLETE
        print(Fore.BLUE + f"Current inchworm state: {self.state}")

    def handle_structure_incomplete(self):
        print(Fore.BLUE + f"IW{self.id}: Structure is incomplete. Updated IW's map with placed block. Finding new path...")
        self.clear_path_com = self.paths
        self.plan_path()
        # TODO: SEND PATH TO STRUCTURE

        self.state = IW_STATE.PATH_PLANNING
        print(Fore.BLUE + f"IW{self.id}: Current inchworm state: {self.state}")

    # Checkers
    def IW_gets_Map_Snapshot(self):
        # blah blah low level language 
        # TODO: ask Mo for help when the IW gets the map SnapShot back 
        # return true if the IW got the map snapshot
        if SIMULATION: 
            if self.leading_foot_loc == self.goal: 
                x, y, z = self.leading_foot_loc
                # if self.current_map[x][y][z] == map_data.GridStatus.WALKABLE.value:
                print(Fore.BLUE + f"IW{self.id}: IW got map snapshot")
                return True
            return False
        else:                
            if not SIMULATION:
                # request the map
                if self.state == IW_STATE.INITIALIZATION and self.request_map_snapshot():
                    return self.inchworm_gets_map()
                else:   
                    return self.inchworm_gets_map()
    
    def is_Path_Available(self):
        print(Fore.BLUE + f"IW{self.id}: Checking path availability... ")
        if self.paths and self.goal:
            return True
        else: 
            return False

    def no_blocks_left(self):
        return self.goal == [-1, -1, -1]

    def is_IW_in_supply(self):
        """ return true if the IW is in the supply location (check the flag and compare the current IW  location through dead reckoning and the supply location)"""
        print(Fore.BLUE + f"IW{self.id}: Checking if at supply location...")
        print(Fore.BLUE + f"IW{self.id}: If in sim, press n to step")
        # TODO: Replace with actual implementation
        if INCHWORM_MOVED:
            IW_in_supply = input("Is iW in supply location? (yes/no) \n")
            if IW_in_supply.lower() == 'yes':
                return True
            elif IW_in_supply.lower() == 'no':
                return False
            else:
                print("Invalid input. Please answer with 'yes' or 'no'.")
        else:
            for bd_loc in BD_LOCS:
                if [bd_loc[0], bd_loc[1], bd_loc[2]-1] == self.leading_foot_loc: 
                    print(Fore.BLUE + f"IW{self.id}: IW thinks it's at the supply depot")
                    return True 
                
            return False

    def is_IW_in_block(self):
        """return true if the IW is in the block location (check the flag and compare the current IW  location through dead reckoning and the block location)"""
        print(Fore.BLUE + f"IW{self.id}: Checking if at block location...")

        # TODO: Replace with actual implementation
        if INCHWORM_MOVED:
            IW_in_supply = input("Is iW in block location? (yes/no) \n")
            if IW_in_supply.lower() == 'yes':
                return True
            elif IW_in_supply.lower() == 'no':
                return False
            else:
                print("Invalid input. Please answer with 'yes' or 'no'.")
        else: 
            if self.leading_foot_loc == self.goal: 
                print(Fore.BLUE + f"IW{self.id}: IW thinks it's at the incoming block loc")
                return True 
            else: 
                return False
        # IW_in_block = input("Is iW in block location? (yes/no) \n")
        # if IW_in_block.lower() == 'yes':
        #     return True
        # elif IW_in_block.lower() == 'no':
        #     return False
        # else:
        #     print("Invalid input. Please answer with 'yes' or 'no'.")

    def incorrect_block_location(self):
        """ return true if the IW gets "incorrectly placed block" from the structure """
        # MOOOO HELLPOPPPP
        if SIMULATION: 
            return not self.leading_foot_loc == self.goal 
        else: 
            # incorrect_block = input("IW got error 'Incorrectly Placed Block'? (yes/no) \n")
            # if incorrect_block.lower() == 'yes':
            #     return True
            # elif incorrect_block.lower() == 'no':
            #     return False
            # else:
            #     print(Fore.BLUE + "Invalid input. Please answer with 'yes' or 'no'.")
            pass
    
    def is_structure_complete(self, curr_map, final_map):
        print(Fore.BLUE + f"IW{self.id}: Checking if structure is complete")

        # compare the current map and the blueprint
        # return true if structure is complete and false otherwise
        
        # for z in range(len(self.current_map[2])):
        #     for x in range(len(self.current_map[1])):
        #         for y in range(len(self.current_map[0])):
        #             if(self.current_map[x][y][z] != self.final_structure[x][y][z]) and self.current_map[x][y][z]:
        #                 return

        curr_map = np.array(curr_map)
        final_map = np.array(final_map)
        # self.current_map = map_data.rm_inchworm_path_from_grid(self.current_map, iw_id=self.id)
        print("current map: ", curr_map)
        print("Final map: ", final_map)
        map_complete = True

        for z in range(curr_map.shape[2]):
            for x in range(curr_map.shape[0]):
                for y in range(curr_map.shape[1]):
                    if curr_map[x, y, z] < 10 and curr_map[x, y, z] != final_map[x, y, z]:
                        print(f"WRONFG THING STUPOIDA ", {x, y, z})
                        map_complete = False

        print("IS MAP COMPLETE: ", map_complete)
        return map_complete
    
        # structure_complete = input("Is structure complete? (yes/no) \n")
        # if structure_complete.lower() == 'yes':
        #     return True
        # elif structure_complete.lower() == 'no':
        #     return False
        # else:
        #     print("Invalid input. Please answer with 'yes' or 'no'.")
        # pass

def step_getter(step_instructions):
    """
    Write the steps to steps.txt
    """
    complete_steps = copy.deepcopy(step_instructions)
    file_path = "steps.txt"
    with open(file_path, 'w') as file:
        for step in complete_steps:
            file.write(f"{step}\n")


if __name__ == "__main__":
        
    with open("/home/smac/robot_ws/src/SMAC6.0/Final_Structure.json", "r") as final_map_file:
        final_structure = json.load(final_map_file)

    inchworm = Inchworm(orientation=IW_ORIENTATIONS[0], final_structure=final_structure, location=IW_LOCS[0], holding_block=False)
    try:
        inchworm.run()
    except KeyboardInterrupt:
        print(Fore.GREEN + "Stopping the inchworm system.") 