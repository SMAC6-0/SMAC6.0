#!/usr/bin/env python3
from lewansoul_servo_bus import ServoBus
from time import sleep, time

servo_bus = ServoBus('/dev/ttyUSB0')

# servo_bus.id_write(4, 2)

mode = int(input("Would you like to read (1) or move (2)?"))

if mode == 1:
    motor_id = int(input("What motor's position would you like to read?"))

    # print(servo_bus.mode_read(int(motor_id)))

    while(True):
        print(servo_bus.pos_read(motor_id))
        sleep(0.5)

elif mode == 2:
    motor_id = int(input("What motor's position would you like to move?"))
    motor_pos = int(input("What position would you like to move the motor to?"))

    currTime = time()
    servo_bus.move_time_write(motor_id, motor_pos, 1)
    while(time() - currTime < 1.0):
        print(servo_bus.pos_read(motor_id))
        sleep(0.1)

elif mode == 3:
    print(servo_bus.angle_limit_write(4, 0, 240))

# print(servo_bus.id_read())