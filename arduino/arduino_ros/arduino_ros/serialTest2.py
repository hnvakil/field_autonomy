import serial
import serial.tools.list_ports as list_ports
import serialWrite
import time,rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

writer = serialWrite.Serial_cmd()

"""
time.sleep(2)
writer.write_data_to_arduino("0.0,0.1\n")
#writer.write_data_to_arduino("\n")

print("past first write")
time.sleep(2)
print("past second sleep")
writer.write_data_to_arduino("0.1,0.0\n")
#writer.write_data_to_arduino("\n")

print("past second write")

time.sleep(2)
print("past third sleep")
writer.write_data_to_arduino("0.0,0.0\n")
#writer.write_data_to_arduino("\n")
"""

str1 = "0.2,0.0>"
str2 = "0.0,0.0>"
count = 0

time.sleep(4)
while True:
    count += 1
    writer.write_data_to_arduino(str1)
    print(str1)
    time.sleep(0.1)
    writer.write_data_to_arduino(str2)
    print(str2)
    time.sleep(0.1)
    print(count)