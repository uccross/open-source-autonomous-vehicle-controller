# -*- coding: utf-8 -*-
import time
from pymavlink import mavutil
import os
import csv
import matplotlib.pyplot as plt
import numpy as np


# Create the connection
# Need to provide the serial port and baudrate
print("Starting application\n")
master = mavutil.mavlink_connection("COM5", baud=57600)
master.wait_heartbeat()
print("target_system {}, target component {} \n", master.target_system,master.target_component)
msg = None
num_points = 2000
data = np.zeros((num_points,3))

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

with open('test_data.csv', 'w', newline='') as csvfile:
    fieldnames = ['mavpackettype', 'time_usec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag', 'id', 'temperature']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    for i in range(num_points):
        try:
            msg = master.recv_match(type='RAW_IMU',blocking = True)
            writer.writerow(msg.to_dict())
            data[i,0] = msg.xacc
            data[i,1] = msg.yacc
            data[i,2] = msg.zacc
            # print(msg.to_dict())
            # if(i % 10 == 0):
            #     xs = msg.xacc
            #     ys = msg.yacc
            #     zs = msg.zacc
                # ax.scatter(xs, ys, zs,color = 'blue')
                # plt.draw()
        except:
            print("msg type exception")

ax.scatter(data[:,0],data[:,1],data[:,2])


