from enum import Enum
import copy
from config import *
import map_data
from inchworm_control.blueprint import blueprint as blueprint
from time import sleep
import serial
import struct
from colorama import Fore, init
init(autoreset=True)

###### UART stuff
UART_BAUD = 9600 # config

# Pins 
# GPIO 15, pin 8 = RX green wire 
# GPIO 14, pin 10 = TX yellow wire
# ground = Pin 14
# test 
DEBUG = True # print statements to help DEBUG 
INCHWORM_MOVED = True # setting this to True so it can bypass all the movements for debugging 

class UART_CODES(Enum):
    StartByte=0xAA 
    Initialization=0xFA
    BeingPlaced=0xFB 
    MapSnapshot=0xFC
    NewBlock=0xFD
    Changes=0xFE
    Failed=0xFF
    NewInchworm=0xEF

class BLOCK_STATUS(Enum): # holds the status of the block 
    Unplaced = 0
    Block = 1
    Placing = 2
    iw_path = 4 

next_block_location = [2, 3, 2] # location of next block, need to change this with blueprint algo dummy valueeee
IW_identifier = 1 # this is the idenifier that goes infornt of the message to be sent to the block 
IW_message_counter = 0 # this is the messgae counter for sending data, IK's message counter increases

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
        # self.found_structures = []
        # self.misc_blocks = []

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

        if DEBUG:
            print("Current Map from IW")
            print(self.current_map)
    def __del__(self):
        """
        Deletion of inchworm in the list of inchworms.
        """
        Inchworm.inchworm_list = [iw for iw in Inchworm.inchworm_list if iw.id != self.id]

    def send_my_next_steps(self): 
        """ Send IW path and the corresponding incoming block to the structure. """ 
        if not SIMULATION: 
            # TODO @ SAKSHI & MO: UART COMMUNICATION
            pass
    
    def plan_path(self, next_goal: tuple[int, int, int] = None): 
        """ Plan path from current location to specified goal. """
        # print(Fore.MAGENTA + f"IW{self.id}, leading: {self.leading_foot_loc}, lagging foot loc: {self.lagging_foot_loc}")

        is_traveling = False # assumes that if not specified, objective is to travel, not place
        if next_goal == None:
            print(Fore.MAGENTA + f"IW{self.id}: goal not given... finding goal now")
            # print(Fore.MAGENTA + "(PP) current_map: ", self.current_map)
            # print(Fore.MAGENTA + "(PP) final_map: ", self.final_structure)
            self.goal = blueprint(self.current_map, self.final_structure) # gets goal from blueprint if none is given
            if self.goal == [-1, -1, -1]:
                print(Fore.MAGENTA + f"IW{self.id}: erm blueprint done in the wrong place")
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
                
                # Find path to where the next block will be placed
                goal_path, goal_steps, new_orientation = map_data.initiate_find_path(self.current_map, bd_path[-2], self.goal, new_orientation, self.holding_block, self.id)
                self.holding_block = False
                goal_path.pop(0) # Remove repeat coord
                
                # combines start to block depot and block depot to goal
                steps += bd_steps
                steps += goal_steps
                path += bd_path
                path += goal_path
            
            # Update inchworm path & corresponding steps to travel that path
            step_instructions += steps
            self.paths += path

            if DEBUG:
                print("path gott yayyyy")

            # Update the inchworm's internal map with the step it will take 
            self.current_map = map_data.set_inchworm_path_to_grid(self.current_map, self.paths, self.id) # Update IW's map with the path
            
            step_getter(step_instructions)
            if DEBUG:
                print("exiting out of pathplannn")

        except RuntimeError as e:
            print(Fore.MAGENTA + f"IW{self.id}: Error: {e}. No path found, try again later.")
            return

    def get_next_point(self): 
        """ 
        Returns the set of the next points of inchworm travel. Used for stepping through path for sim.
        """
        if self.goal_progress_index > 0:
            self.leading_foot_loc = self.paths[self.goal_progress_index]  # Get the next point
            self.lagging_foot_loc = self.paths[self.goal_progress_index - 1]
        
        x, y, z = self.leading_foot_loc
        self.goal_progress_index += 1
        
        if ([x, y, z] == [BD_1_LOC[0], BD_1_LOC[1], BD_1_LOC[2]-1]):
            self.holding_block = True
        elif self.holding_block & ([x, y, z] == [self.goal[0], self.goal[1], self.goal[2]-1]):
            self.holding_block = False
        
        if self.holding_block and [x, y, z] != self.goal:
            z = z + 1
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
        block_change = struct.pack('B', IW_identifier) # indicate that an inchworm is sending this message

        for c in next_block_location:
            block_change += struct.pack('B', c)

        block_change += struct.pack('B', BLOCK_STATUS.Unplaced.value) + struct.pack('B', IW_message_counter)

        # print(Fore.RED + "Block change", block_change)

        # calculate message length and checksum

        msg_len = len(block_change).to_bytes(2,'little')
        checksum = self.crc16(block_change).to_bytes(2, 'little')

        # print(Fore.RED + "msg_len", msg_len)
        # print(Fore.RED + "checksum", checksum)

        # append msg_len, block_change, checksum, ending_code(enum) to buffer

        buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.Initialization.value)

        self.IW_SERIAL.write(buffer)
        print(Fore.RED + "block data sent!!")
        # TODO: handle transmission error

    def send_block_being_placed(self):
        """
        IW sends the Hex code back to the block indicating that it's being placed
        """
        buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

        # block_change is the data that needs to be sent
        block_change = struct.pack('B', IW_identifier) # indicate that an inchworm is sending this message

        for c in next_block_location:
            block_change += struct.pack('B', c)

        block_change += struct.pack('B', BLOCK_STATUS.Placing.value) + struct.pack('B', IW_message_counter)

        # print(Fore.RED + "Block change", block_change)

        # calculate message length and checksum

        msg_len = len(block_change).to_bytes(2,'little')
        checksum = self.crc16(block_change).to_bytes(2, 'little')

        # print(Fore.RED + "msg_len", msg_len)
        # print(Fore.RED + "checksum", checksum)

        # append msg_len, block_change, checksum, ending_code(enum) to buffer

        buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.BeingPlaced.value)

        self.IW_SERIAL.write(buffer)

        print(Fore.RED + "Indicated block is in placing status!!")
        # TODO: handle transmission error

    def received_block_confirmation(self): 
        print(Fore.RED + "We're trying to confirm the block's existence & ability to communicate, but we haven't been implemented yet D:")
        return True
    
    def send_IW_path_to_block(self, iw_path):
        """
        Sends the IW path to the structure one grid at a time 

        Args: 
            iw_path [list[list]]: the path of the inchworm 
        """
        # TODO: IW_path is in X, Y, Z format!!
        # iterate through the iw_path
        for grid_cell in iw_path:
            buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

            # block_change is the data that needs to be sent
            block_change = struct.pack('B', IW_identifier) # indicate that an inchworm is sending this message

            for c in grid_cell:
                block_change += struct.pack('B', c)

            block_change += struct.pack('B', BLOCK_STATUS.iw_path.value) + struct.pack('B', IW_message_counter)

            msg_len = len(block_change).to_bytes(2,'little')
            checksum = Inchworm.crc16(block_change).to_bytes(2, 'little')

            # append msg_len, block_change, checksum, ending_code(enum) to buffer

            buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.Changes.value)
            print("buffer", buffer)

            self.IW_SERIAL.write(buffer)

            if DEBUG:
                print("grid celllllll RAHHH")

            # delay to make sure all the data is transmitted 
            sleep(PATH_PLANNING_TIMER)

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

            if byte == bytearray(struct.pack('B', UART_CODES.StartByte.value)) and collecting_data == False:  # Start byte detected
                # print("start byte detected")
                buffer = []  
                bytesRead = 0
                msgLenCollected = False
                msgLenBytes = []
                msgLenReceivedCounter = 0
                collecting_data = True
                
            elif not msgLenCollected and msgLenReceivedCounter < 2: # Collecting Message Length
                # print("Collecting Message Length")
                # byte_in_int = ord(byte)
                msgLenBytes.append(byte[0])
                msgLenReceivedCounter += 1
                if msgLenReceivedCounter == 2:
                    # Convert collected bytes to integer (assuming big-endian format)
                    msgLen = int.from_bytes(bytes(msgLenBytes), 'little')
                    msgLenCollected = True

            elif byte == bytearray(struct.pack('B', UART_CODES.MapSnapshot.value)) and  bytesRead >= msgLen: # Receiving Map Snapshot from Structure
                if collecting_data:
                    buffer = b''.join(buffer) # convert to bytes object
                    checksum = Inchworm.get_checksum(buffer)
                    calculated_check_sum = []
                    calculated_check_sum += Inchworm.crc16(buffer[:-2]).to_bytes(2, 'little')
                    calculated_check_sum = int.from_bytes(bytes(calculated_check_sum), 'big')
                    # Inchworm.crc16(buffer[:-2])
                    
                    if checksum == calculated_check_sum:
                        self.current_map = Inchworm.process_received_map_snapshot(buffer)
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
        layers, rows, cols = 4, 8, 8
        array = [[[0 for _ in range(cols)] for _ in range(rows)] for _ in range(layers)]
        index = 0
        for l in range(layers):
            for r in range(rows):
                for c in range(cols):
                    if index < len(map_data):
                        array[l][r][c] = map_data[index]
                        index += 1
        print("Received 3D Array:", array)
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
                    if self.is_structure_complete(): # structure is complete
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
        if self.paths: # this happens second 
            # move IW in sim
            if DEBUG:
                print("IW has paths")
            if self.goal_progress_index >= len(self.paths) or INCHWORM_MOVED: 
                print(Fore.BLUE + "Touching the seed block")
                self.current_map = map_data.rm_inchworm_path_from_grid(self.current_map, self.paths)
                self.paths = [] # Reset current path 
                self.goal_progress_index = 0 # TODO: May be good to move to handle_IW_gets_Map or clear_my_path

                print(Fore.BLUE + "Reset the path.")

        else: # this happens first 
            # Find & path plan to seed block 
            if DEBUG:
                print("IW has no paths so path planning")
            self.plan_path(SEED_BK)

        
        
    def handle_IW_gets_Map(self):
        print(Fore.BLUE + f"IW{self.id}: Map snapshot successful. Now path planning...")
        if DEBUG:
            print("Current Map from Block")
            print(self.current_map)

        self.plan_path()
        self.state = IW_STATE.PATH_PLANNING
        print(Fore.BLUE + f"Current inchworm state: {self.state}")

    def path_exists(self):
        # MOOOO HELPPP 
        print(Fore.BLUE + f"IW{self.id}: Path found. Sending the IW path to the structure")
        # IW sends it's path to the structure 
        if not SIMULATION:
            self.send_IW_path_to_block(self.paths)
        
        print("")
        self.state = IW_STATE.TRAVELLING_TO_SUPPLY
        print(Fore.BLUE + f"Current inchworm state: {self.state}")
    
    def retry_path(self):
        # Question: Is it ok for the IW to sleep?!! cuz then it doesn't get active data yk 
        if not SIMULATION:
            sleep(PATH_PLANNING_TIMER) # TODO: Decide if we need a  sleep here because we want to have a non blocking code
        # or stay here until the IW gets a new map!!
        # MOOO HELPPP
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
            sleep(PATH_PLANNING_TIMER) # TODO: Decide if we need a  sleep here because we want to have a non blocking code

            if self.received_block_confirmation():
                print(Fore.BLUE + f"IW{self.id}: IW sends a messgae indicating block is being placed")
                # IW sends a messgae indicating block is being placed
                # MOOOOO HELPPPP
                self.send_block_being_placed()
            else: 
                self.handle_error()
        
        self.state = IW_STATE.TRANSPORTING_BLOCK
        print(Fore.BLUE + f"IW{self.id}: Current inchworm state: {self.state}")

    def handle_transported_block(self):
        # self.current_map = map_data.rm_inchworm_path_from_grid(self.current_map, self.paths)
        self.paths = [] # Reset current path 
        self.goal_progress_index = 0 # TODO: May be good to move to handle_IW_gets_Map or clear_my_path
        self.holding_block = False

        print(Fore.BLUE + f"IW{self.id}: Reset the path")

        self.state = IW_STATE.PLACING_BLOCK
        print(Fore.BLUE + f"IW{self.id}: Current inchworm state: {self.state}")

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
                if self.current_map[x][y][z] == map_data.GridStatus.WALKABLE.value:
                    print(Fore.BLUE + f"IW{self.id}: IW got map snapshot")
                    return True
            return False
        else:                
            if not SIMULATION:
                # request the map
                if self.request_map_snapshot():
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
        for bd_loc in BD_LOCS:
            if [bd_loc[0], bd_loc[1], bd_loc[2]-1] == self.leading_foot_loc: 
                print(Fore.BLUE + f"IW{self.id}: IW thinks it's at the supply depot")
                return True 
             
        return False


    def is_IW_in_block(self):
        """return true if the IW is in the block location (check the flag and compare the current IW  location through dead reckoning and the block location)"""
        print(Fore.BLUE + f"IW{self.id}: Checking if at block location...")

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
    
    def is_structure_complete(self):
        print(Fore.BLUE + f"IW{self.id}: Checking if structure is complete")

        # compare the current map and the blueprint
        # return true if structure is complete and false otherwise
        return self.current_map == self.final_structure
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
        
    # file_path = '/home/smac/robot_ws/src/SMAC6.0/Final_Structure.txt'
    with open("/home/smac/robot_ws/src/SMAC6.0/Final_Structure.txt", "r") as file:
      final_structure = file.readlines()

      
    print("I'm hereee")
    inchworm = Inchworm(orientation=IW_ORIENTATIONS[0], final_structure=final_structure, location=IW_LOCS[0], holding_block=False)
    try:
        inchworm.run()
    except KeyboardInterrupt:
        print(Fore.GREEN + "Stopping the inchworm system.") 