#!/usr/bin/env python3
from lewansoul_servo_bus import ServoBus
from time import sleep

servo_bus = ServoBus('/dev/ttyUSB0')

# servo_bus.id_write(1, 3)

motor_id = input("What motor's position would you like to read?")

print(servo_bus.mode_read(int(motor_id)))

# while(True):
#     print(servo_bus.pos_read(int(motor_id)))
#     sleep(0.5)
# print(servo_bus.id_read())