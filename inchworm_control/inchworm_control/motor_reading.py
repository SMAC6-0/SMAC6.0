#!/usr/bin/env python3
from lewansoul_servo_bus import ServoBus

servo_bus = ServoBus('/dev/ttyUSB0')

motor_id = input("What motor's position would you like to read?")

print(servo_bus.pos_read(int(motor_id)))
# print(servo_bus.id_read())