#!/usr/bin/env python3
from lewansoul_servo_bus import ServoBus

servo_bus = ServoBus('/dev/ttyUSB0')

print(servo_bus.id_read())