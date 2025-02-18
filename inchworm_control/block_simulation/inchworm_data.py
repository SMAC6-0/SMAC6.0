from enum import Enum
import copy
from config import *
import map_data
from inchworm_control.blueprint import blueprint as blueprint
from time import sleep
import time
import serial
import struct

###### UART stuff
UART_BAUD = 9600 # config

# Pins 
# GPIO 15, pin 8 = RX green wire 
# GPIO 14, pin 10 = TX yellow wire
# ground = Pin 14
# test 

class UART_CODES(Enum):
    StartByte=0xAA 
    Initialization=0xFA
    BeingPlaced=0xFB 
    MapSnapshot=0xFC
    NewBlock=0xFD
    Changes=0xFE
    Failed=0xFF

class BLOCK_STATUS(Enum): # holds the status of the block 
    Unplaced = 0
    Placing = 2

SUPPLY_LOCATION = [1, 1, 1] # config

# TODO: sakshi this you? is it in xyz?
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

PATH_PLANNING_TIMER = 3

lagging_transform = {
    InchwormOrientation.NORTH: lambda x, y, z: (x, y, z-1),  
    InchwormOrientation.SOUTH: lambda x, y, z: (x, y, z+1),
    InchwormOrientation.EAST: lambda x, y, z: (x-1, y, z), 
    InchwormOrientation.WEST: lambda x, y, z: (x+1, y, z) 
}

class Inchworm:
    next_id = 1
    inchworm_list = []
    
    def __init__(self, orientation, final_structure, location: tuple[int]=CURRENT_LOC, holding_block=False):
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
        self.initilization_flag = True
        self.print_flag = True
        self.seed_block_flag = True

        
        Inchworm.next_id += 1
        Inchworm.inchworm_list.append(self)

        # UART stuff
        if not SIMULATION: 
            self.IW_SERIAL = serial.Serial ("/dev/ttyAMA0", 9600)    #Open port with baud rate
    
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

    def get_loc_in_path(self): 
        return tuple(map(float, self.goal))
    
    def plan_path(self, next_goal: tuple[int, int, int] = None): 
        """ Plan path from current location to specified goal. """
        is_traveling = False # assumes that if not specified, objective is to travel, not place
        if next_goal == None:
            print("goal not given... finding goal now")
            # print("(PP) current_map: ", self.current_map)
            # print("(PP) final_map: ", self.final_structure)
            self.goal = blueprint(self.current_map, self.final_structure) # gets goal from blueprint if none is given
            if self.goal == [-1, -1, -1]:
                print("erm blueprint done in the wrong place")
                return
            elif self.goal == [-9, -9, -9]:
                raise ValueError(f"Erm... No goal was given... No structure was found...")
            
            if [self.goal[0], self.goal[1], self.goal[2]+1] != SEED_BK:
                print("Setting IW's goal to be incoming block")
                self.current_map = map_data.update_grid_status(self.current_map, self.goal, map_data.GridStatus.INCOMING_BLOCK) # updates map for next_goal to be incoming_block
        else:
            is_traveling = True
            self.goal = next_goal
            if self.goal != SEED_BK: 
                print("Goal is not seed block. Setting IW's goal to be incoming block")
                self.current_map = map_data.update_grid_status(self.current_map, self.goal, map_data.GridStatus.INCOMING_BLOCK) # updates map for next_goal to be incoming_block

        
        # print("goal: ", self.goal)
        # x, y, z =self.goal
        # print("IW's evaluation of goal. Below:  ", self.current_map[x][y][z-1], " itself: ", self.current_map[x][y][z], " above: ", self.current_map[x][y][z+1])

        try: 
            step_instructions, steps, path = [], [], []
            if is_traveling or self.holding_block:
                # Find one path, to travel to the specified goal
                path, steps = map_data.initiate_find_path(self.current_map, self.leading_foot_loc, self.goal, self.orientation, self.holding_block, self.id)
            else:
                # Find path to block depot 
                bd_path, bd_steps = map_data.initiate_find_path(self.current_map, self.leading_foot_loc, BD_LOC1, self.orientation, self.holding_block, self.id)
                self.holding_block = True
                
                # Find path to where the next block will be placed
                goal_path, goal_steps = map_data.initiate_find_path(self.current_map, BD_LOC1, self.goal, self.orientation, self.holding_block, self.id)
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

            # Update the inchworm's internal map with the step it will take 
            self.current_map = map_data.set_inchworm_path_to_grid(self.current_map, self.paths, self.id) # Update IW's map with the path
            
            step_getter(step_instructions)
        except RuntimeError as e:
            print(f"Error: {e}. No path found, try again later.")
            return

    def get_next_point(self): 
        """ 
        Returns the set of the next points of inchworm travel. Used for stepping through path for sim.
        """
        self.leading_foot_loc = self.paths[self.goal_progress_index]  # Get the next point
        x, y, z = self.leading_foot_loc
        self.goal_progress_index += 1
        
        if ([x, y, z] == [BD_LOC1[0], BD_LOC1[1], BD_LOC1[2]-1]):
            self.holding_block = True
        elif self.holding_block & ([x, y, z] == [self.goal[0], self.goal[1], self.goal[2]-1]):
            self.holding_block = False
        
        if self.holding_block and [x, y, z] != self.goal:
            z = z + 1
        return x, y, z # !!!change because interacting with sim.py!!!
    
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

        print("Block change", block_change)

        # calculate message length and checksum

        msg_len = len(block_change).to_bytes(2,'little')
        checksum = self.crc16(block_change).to_bytes(2, 'little')

        print("msg_len", msg_len)
        print("checksum", checksum)

        # append msg_len, block_change, checksum, ending_code(enum) to buffer

        buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.Initialization.value)

        self.IW_SERIAL.write(buffer)
        print("block data sent!!")
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

        print("Block change", block_change)

        # calculate message length and checksum

        msg_len = len(block_change).to_bytes(2,'little')
        checksum = self.crc16(block_change).to_bytes(2, 'little')

        print("msg_len", msg_len)
        print("checksum", checksum)

        # append msg_len, block_change, checksum, ending_code(enum) to buffer

        buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.BeingPlaced.value)

        self.IW_SERIAL.write(buffer)

        print("Indicated block is in placing status!!")
        # TODO: handle transmission error


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
                if self.initilization_flag:
                    self.handle_initilization()
                if self.IW_gets_Map_Snapshot(): # IW got the mapsnap shot 
                    self.handle_IW_gets_Map()
            case IW_STATE.PATH_PLANNING:
                if self.is_Path_Available(): # Path exists!
                    self.path_exists()
                else: # Path doesn't exist!
                    print("Retrying path planning after waiting")
                    self.retry_path() 
            case IW_STATE.TRAVELLING_TO_SUPPLY:
                if self.is_IW_in_supply():
                    self.handle_at_supply()
                # elif not on path, then handle error????
                # else: # TODO: Commented out bc the IW has to step multiple times before its at the supply, it'd default to thinking it's in error
                #     self.handle_error()
            case IW_STATE.TRANSPORTING_BLOCK:
                if self.is_IW_in_block():
                    self.handle_transported_block()
                # else:
                #     self.handle_error()
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
            print("IDLINGGG....")
            self.print_flag = False

    
    # during the initiliaztion phase the inchworm should lift up it's gripper and touch the seed block
    # and transfer the block location to the seed block
    def handle_initilization(self):
        print("MOVINGGG TO SEED BLOCK: press n to step")
        if self.paths: # this happens second 
            # move IW in sim
            if self.goal_progress_index >= len(self.paths): 
                print("Touching the seed block")
                self.initilization_flag = False 
                self.current_map = map_data.rm_inchworm_path_from_grid(self.current_map, self.paths)
                self.paths = [] # Reset current path 
                self.goal_progress_index = 0 # TODO: May be good to move to handle_IW_gets_Map or clear_my_path

                print("Reset the path. Transferring the block data")
                # transfer the block location data 
                # TODO: MOOO help 
                # send a 1D array ended with the Initialization enum OxFA 
                # flash block that it's in unplaced location

        else: # this happens first 
            # Find & path plan to seed block 
            self.plan_path(SEED_BK)

        
        
    def handle_IW_gets_Map(self):
        print("Map snapshot successful. Now path planning...")
        self.plan_path()
        self.state = IW_STATE.PATH_PLANNING
        print(f"Current inchworm state: {self.state}")

    def path_exists(self):
        # MOOOO HELPPP 
        print("Path found. Sending the IW path to the structure")
        # IW sends it's path to the structure 
        self.send_my_next_steps()
        # TODO !!!!! 
        print("Travelling to the supply")
        self.state = IW_STATE.TRAVELLING_TO_SUPPLY
        print(f"Current inchworm state: {self.state}")
    
    def retry_path(self):
        # Question: Is it ok for the IW to sleep?!! cuz then it doesn't get active data yk 
        sleep(PATH_PLANNING_TIMER) # TODO: Decide if we need a  sleep here because we want to have a non blocking code
        # or stay here until the IW gets a new map!!
        # MOOO HELPPP
        self.plan_path()

    def handle_at_supply(self):
        print("Touching the new block (move)")
        # touch the new block
        if not SIMULATION: 
            print("IW flashes block with it's location")
            self.send_block_location()

            # pause so that the block has enough time to process the info
            sleep(PATH_PLANNING_TIMER) # TODO: Decide if we need a  sleep here because we want to have a non blocking code


            print("IW sends a messgae indicating block is being placed")
            # IW sends a messgae indicating block is being placed
            # MOOOOO HELPPPP
            self.send_block_being_placed()


        print("Travelling to the block location")
        # IW begins travelling to block location
        
        self.state = IW_STATE.TRANSPORTING_BLOCK
        print(f"Current inchworm state: {self.state}")

    def handle_transported_block(self):

        self.current_map = map_data.rm_inchworm_path_from_grid(self.current_map, self.paths)
        self.paths = [] # Reset current path 
        self.goal_progress_index = 0 # TODO: May be good to move to handle_IW_gets_Map or clear_my_path
        x, y, z =self.goal
        print("IW's evaluation of goal. Below:  ", self.current_map[x][y][z-1], " itself: ", self.current_map[x][y][z], " above: ", self.current_map[x][y][z+1])
        self.holding_block = False

        print("Reset the path")

        self.state = IW_STATE.PLACING_BLOCK
        print(f"Current inchworm state: {self.state}")

    def handle_error(self):
        print("OHHH NOOO, ERROR ERROR")

        print("Stop Inchworm")
        # stop the inchworm
        
        print("flash red LED")
        # flash red Led 

        self.state = IW_STATE.IDLE
        print(f"Current inchworm state: {self.state}")
    
    def handle_structure_complete(self):
        print("Structure is complete YIppeee")

        # stop the iW
        print("IW Stopped")
        
        # flash green light
        print("flash Green LED")

        self.state = IW_STATE.STRUCTURE_COMPLETE
        print(f"Current inchworm state: {self.state}")

    def handle_structure_incomplete(self):
        print("Structure is incomplete. Updated IW's map with placed block. Finding new path...")
        self.plan_path()
        # TODO: SEND PATH TO STRUCTURE
        print("current IW: ", self.current_map)
        print("final map: ", self.final_structure)

        self.state = IW_STATE.PATH_PLANNING
        print(f"Current inchworm state: {self.state}")

    # Checkers
    def IW_gets_Map_Snapshot(self):
        # blah blah low level language 
        # TODO: ask Mo for help when the IW gets the map SnapShot back 
        # return true if the IW got the map snapshot
        if SIMULATION: 
            if self.leading_foot_loc == self.goal: 
                x, y, z = self.leading_foot_loc
                if self.current_map[x][y][z] == map_data.GridStatus.WALKABLE.value:
                    print("IW got map snapshot")
                    return True
            return False
        else:             
            if self.seed_block_flag: # skip the seed block since Mo has to implement this in the block communication
                got_map_snapshot = input("Did the inchworm get the map? (seed block) (yes/no): \n")
                if got_map_snapshot.lower() == 'yes':
                    self.seed_block_flag = False
                    return True
                elif got_map_snapshot.lower() == 'no':
                    return False
                else:
                    print("Invalid input. Please answer with 'yes' or 'no'.")
            else:
                # blah blah low level language 
                # TODO: ask Mo for help when the IW gets the map SnapShot back 
                # return true if the IW got the map snapshot
                received_data = self.IW_SERIAL.read()              #read serial port
                sleep(0.03)
                data_left = self.IW_SERIAL.inWaiting()             #check for remaining byte
                received_data += self.IW_SERIAL.read(data_left)
                print (received_data)                   #print received data

                # verify if it's a map?? 
                is_a_map = True
                if is_a_map:
                    # call the update map
                    # self.current_map = map_data.update_grid_status(self.current_map, coord)
                    pass
                # return is_a_map
    
    def is_Path_Available(self):
        print("Checking path availability... ")
        if self.paths and self.goal:
            return True
        else: 
            return False


    def is_IW_in_supply(self):
        """ return true if the IW is in the supply location (check the flag and compare the current IW  location through dead reckoning and the supply location)"""
        print("Checking if at supply location...")
        print("If in sim, press n to step")
        for bd_loc in BD_LOCS:
            if [bd_loc[0], bd_loc[1], bd_loc[2]-1] == self.leading_foot_loc: 
                print("IW thinks it's at the supply depot")
                return True 
            else: 
                return False


    def is_IW_in_block(self):
        """return true if the IW is in the block location (check the flag and compare the current IW  location through dead reckoning and the block location)"""
        print("Checking if at block location...")

        if self.leading_foot_loc == self.goal: 
            print("IW thinks it's at the incoming block loc")
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
            incorrect_block = input("IW got error 'Incorrectly Placed Block'? (yes/no) \n")
            if incorrect_block.lower() == 'yes':
                return True
            elif incorrect_block.lower() == 'no':
                return False
            else:
                print("Invalid input. Please answer with 'yes' or 'no'.")
            pass
    
    def is_structure_complete(self):
        print("Checking if structure is complete")

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
    inchworm = Inchworm(1, CURRENT_ORIENTATION, None, None, CURRENT_LOC)
    try:
        inchworm.run()
    except KeyboardInterrupt:
        print("Stopping the inchworm system.") 