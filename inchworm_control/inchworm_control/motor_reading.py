#!/usr/bin/env python3
from lewansoul_servo_bus import ServoBus
from time import sleep, time

servo_bus = ServoBus('/dev/ttyUSB0')

# servo_bus.id_write(4, 2)

# motor_id = input("What motor's position would you like to read?")

# print(servo_bus.mode_read(int(motor_id)))

# while(True):
#     print(servo_bus.pos_read(int(motor_id)))
#     sleep(0.5)

motor_pos = input("What position would you like to move the motor to?")

currTime = time()
servo_bus.move_time_write(5, int(motor_pos), 1)
while(time() - currTime < 1.0):
    print(servo_bus.pos_read(5))
    sleep(0.25)

# print(servo_bus.id_read())