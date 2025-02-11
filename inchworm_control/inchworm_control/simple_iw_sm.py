from enum import Enum
import time
import serial
import struct
from time import sleep

###### UART stuff
UART_BAUD = 9600 # config

# Pins 
# GPIO 15, pin 8 = RX green wire 
# GPIO 14, pin 10 = TX yellow wire
# ground = Pin 14

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

next_block_location = [2, 3, 2] # location of next block, need to change this with blueprint algo dummy valueeee
IW_identifier = 1 # this is the idenifier that goes infornt of the message to be sent to the block 
IW_message_counter = 0 # this is the messgae counter for sending data, IK's message counter increases

IW_PATH = [[4, 0, 1], [5, 0, 1], [6, 0, 1], [7, 0, 1], [7, 0, 2], [7, 0, 3], [7, 0, 4], [7, 0, 5], [7, 0, 6], [7, 0, 7], [7, 1, 8]]

# final_map = [
#     [ # X = 0
#         [0, 0, 0, 0, 0, 0, 0, 0], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1]
#     ], 
#     [ #x=1
#         [0, 0, 0, 0, 0, 0, 0, 0], #z=0
#         [1, 1, 1, 1, 1, 1, 1, 1], #z=1
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1]
#     ], 
#     [
#         [0, 0, 0, 0, 0, 0, 0, 0], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 0, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1]
#     ], 
#     [
#         [0, 0, 0, 0, 0, 0, 0, 0], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 0, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1]
#     ], 
#     [
#         [0, 0, 0, 0, 3, 0, 0, 0], 
#         [1, 1, 1, 1, 0, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1]
#     ], 
#     [
#         [0, 0, 0, 0, 0, 0, 0, 0], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1]
#     ], 
#     [
#         [0, 0, 0, 0, 0, 0, 0, 0], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1]
#     ], 
#     [
#         [0, 0, 0, 0, 0, 0, 0, 0], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1], 
#         [1, 1, 1, 1, 1, 1, 1, 1]
#     ]
# ]

current_map = [
    [  # Layer 0
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ],
    [  # Layer 1
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ],
    [  # Layer 2
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ]
]



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

class Inchworm:
    def __init__(self):
        self.state = IW_STATE.INITIALIZATION
        self.initilization_flag = True
        self.print_flag = True
        self.seed_block_flag = True
        self.retry_path_flag = True


        # UART stuff
        self.IW_SERIAL = serial.Serial ("/dev/ttyAMA0", 9600)    #Open port with baud rate

    def run(self):
        while self.state != IW_STATE.STRUCTURE_COMPLETE:
            self.update_state()
    def update_state(self):
        match self.state:
            case IW_STATE.IDLE:
                self.handle_idle()
            case IW_STATE.INITIALIZATION:
                if self.initilization_flag:
                    self.handle_initialization()

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
                else:
                    self.handle_error()
            case IW_STATE.TRANSPORTING_BLOCK:
                if self.is_IW_in_block_location():
                    self.handle_transported_block()
                else:
                    self.handle_error()
            case IW_STATE.PLACING_BLOCK:
                if self.incorrect_block_location(): # blocto_bytes(2, 'little')tructure is complete
                    self.handle_error()
                if self.is_structure_complete():
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
    def handle_initialization(self):
        print("MOVINGGG...")
        # path plan to the seed block location from the supply depot
        
        # touch the block infornt of it

        print("Initializing the block")

        # different for the seed block logic
        # TODO: update this logic after Mo implements his seed block logic 
        print("Sending data about the seed block")
        
        '''
        # transfer the block location data 
        # TODO: MOOO help 
        # send a 1D array ended with the Initialization enum OxFA 
        # flash block that it's in unplaced location

        buffer = bytearray(struct.pack('B', 0xAA)) # universal start code

        # block_change is the data that needs to be sent
        block_change = struct.pack('B', IW_identifier) # indicate that an inchworm is sending this message

        for c in next_block_location:
            block_change += struct.pack('B', c)

        block_change += struct.pack('B', BLOCK_STATUS.Unplaced) + struct.pack('B', IW_message_counter)

        print("Block change", block_change)

        # calculate message length and checksum

        msg_len = len(block_change).to_bytes(2,'little')
        checksum = self.crc16(block_change).to_bytes(2, 'little')

        print("msg_len", msg_len)
        print("checksum", checksum)

        # append msg_len, block_change, checksum, ending_code(enum) to buffer

        buffer += msg_len + block_change + checksum + struct.pack('B', 0xFA)

        self.IW_SERIAL.write(buffer)
        '''
        
        # TODO IW gets the error messgae back if the transmission is failed 

        # print("sent data yippee")
        self.initilization_flag = False
        
    def handle_IW_gets_Map(self):
        print("Map snapshot successful.")

        self.state = IW_STATE.PATH_PLANNING
        print(f"Current inchworm state: {self.state}")

    def path_exists(self):
        # MOOOO HELPPP 
        if self.seed_block_flag:
            print("Sending the IW path to the structure (seed block)")
            # IW sends it's path to the structure 
        else:
            IW_PATH

        print("Travelling to the supply")
        # IW begins travelling to supply location

        self.state = IW_STATE.TRAVELLING_TO_SUPPLY
        print(f"Current inchworm state: {self.state}")
    
    def retry_path(self):
        # Question: Is it ok for the IW to sleep?!! cuz then it doesn't get active data yk 
        sleep(PATH_PLANNING_TIMER) # TODO: Decide if we need a  sleep here because we want to have a non blocking code
        # or stay here until the IW gets a new map!!
        # TODO: call map snapshot? 
        # if self.IW_gets_Map_Snapshot():
        #     print("IW got map snap shot, retry again")
        # MOOO HELPPP

    def handle_at_supply(self):
        print("Touching the new block (move)")
        # touch the new block

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
        print("Structure is incomplete")

        self.state = IW_STATE.PATH_PLANNING
        print(f"Current inchworm state: {self.state}")

    # Checkers
    def IW_gets_Map_Snapshot(self):
        if self.seed_block_flag: # skip the seed block since Mo has to implement this in the block communication
            got_map_snapshot = input("Did the inchworm get the map? (seed block) (yes/no): \n")
            if got_map_snapshot.lower() == 'yes':
                # self.seed_block_flag = False
                return True
            elif got_map_snapshot.lower() == 'no':
                return False
            else:
                print("Invalid input. Please answer with 'yes' or 'no'.")
        else:
            # blah blah low level language 
            # TODO: ask Mo for help when the IW gets the map SnapShot back  q1
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
                self.update_my_current_map()
            # return is_a_map

            # layers, rows, cols = 3, 5, 6
            # array = [[[0 for _ in range(cols)] for _ in range(rows)] for _ in range(layers)]
            # index = 0
            # for l in range(layers):
            #     for r in range(rows):
            #         for c in range(cols):
            #             if index < len(map_data):
            #                 array[l][r][c] = map_data[index]
            #                 index += 1
            # return array  
            # print("Received 3D Array:", array)
    
    def is_Path_Available(self):
        # # question how do we know if this path is the most upto date path
        # return not IW_Path == [] # return if IW_path is empty or not (True: if not empty)
        # TODO: add the path planning stuff 
        print("Planning path from supply to the next block")
        # IW path plans to the supply and to the next block
        # store that path in IW_path 

        print("Checking path availability...")
        is_Path_Available = input("Is Path Available? (yes/no) \n")
        if is_Path_Available.lower() == 'yes':
            return True
        elif is_Path_Available.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")

    def is_IW_in_supply(self):
        # return true if the IW is in the supply location (check the flag and compare the current IW  location through dead reckoning and the supply location)
        print("Checking if at supply location...")

        IW_in_supply = input("Is iW in supply? (yes/no) \n")
        if IW_in_supply.lower() == 'yes':
            return True
        elif IW_in_supply.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")

    def is_IW_in_block_location(self):
        #  return true if the IW is in the block location (check the flag and compare the current IW  location through dead reckoning and the block location)
        print("Checking if at block location...")

        IW_in_block = input("Is iW in block location? (yes/no) \n")
        if IW_in_block.lower() == 'yes':
            return True
        elif IW_in_block.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")

    def incorrect_block_location(self):
        # return true if the IW gets "incorrectly placed block" from the structure 
        # MOOOO HELLPOPPPP

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

        structure_complete = input("Is structure complete? (yes/no) \n")
        if structure_complete.lower() == 'yes':
            return True
        elif structure_complete.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")
        pass
    
    # other functionsss

    def update_my_current_map(self, map): 
        """
        Updates the inchworm's map based on received updates from the structure. 
        Args: 
            map: xzy (3D) list storing the current status of the map, as the structure knows it.  
        """
        # TODO: does this belong in checker, handler, or outside? @Mo 

        # map updates so we need to manually update the x, z, y
        # self.current_map = map
        print("Current map updated")
        pass
    
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

        # print("Block change", block_change)

        # calculate message length and checksum

        msg_len = len(block_change).to_bytes(2,'little')
        checksum = self.crc16(block_change).to_bytes(2, 'little')

        # print("msg_len", msg_len)
        # print("checksum", checksum)

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

        # print("Block change", block_change)

        # calculate message length and checksum

        msg_len = len(block_change).to_bytes(2,'little')
        checksum = self.crc16(block_change).to_bytes(2, 'little')

        # print("msg_len", msg_len)
        # print("checksum", checksum)

        # append msg_len, block_change, checksum, ending_code(enum) to buffer

        buffer += msg_len + block_change + checksum + struct.pack('B', UART_CODES.BeingPlaced.value)

        self.IW_SERIAL.write(buffer)

        print("Indicated block is in placing status!!")
        # TODO: handle transmission error

    def send_IW_path_to_block(self, iw_path):
        """
        Sends the IW path to the structure one grid at a time 

        Args: 
            iw_path [list[list]]: the path of the inchworm 
        """
        # iterate through the iw_path
        for grid in range(len(iw_path)):
            print("grid", grid)
            buffer = bytearray(struct.pack('B', UART_CODES.StartByte.value)) # universal start code

            # block_change is the data that needs to be sent
            block_change = struct.pack('B', IW_identifier) # indicate that an inchworm is sending this message





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
    
    
# an instance of Inchworm Statemachine
inchworm_sm = Inchworm()

# Simulate the state machine
def run_Inchworm ():
    print("Current inchworm state:")

if __name__ == "__main__":
    inchworm = Inchworm()
    try:
        inchworm.run()
    except KeyboardInterrupt:
        print("Stopping the inchworm system.")