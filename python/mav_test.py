# -*- coding: utf-8 -*-
import time
# Import mavutil
from pymavlink import mavutil
import os
import csv


# Create the connection
# Need to provide the serial port and baudrate
print("Starting application\n")
master = mavutil.mavlink_connection("COM5", baud=57600)
master.wait_heartbeat()
print("target_system {}, target component {} \n", master.target_system,master.target_component)
msg = None


with open('test_data.csv', 'w', newline='') as csvfile:
    fieldnames = ['mavpackettype', 'time_usec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag', 'id', 'temperature']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    for i in range(1000):
        try:
            msg = master.recv_match(type='RAW_IMU',blocking = True)
            writer.writerow(msg.to_dict())
            print(msg.to_dict())
        except:
            print("msg type exception")

    

