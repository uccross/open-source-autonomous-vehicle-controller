# -*- coding: utf-8 -*-
import time
from pymavlink import mavutil
import os
import csv
import matplotlib.pyplot as plt
import numpy as np

def mav_log_imu(num_points, file = 'test_data.csv'):
    '''logs IMU data from a MAVLINK message stream'''
    with open(file, 'w', newline='') as csvfile:
        # fieldnames = ['mavpackettype', 'time_usec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag', 'id', 'temperature']
        fieldnames = ['mavpackettype', 'time_usec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag', 'abs_pressure', 'diff_pressure', 'pressure_alt', 'temperature', 'fields_updated', 'id' ]        
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)    
        writer.writeheader()
        for i in range(num_points):
            try:
                msg = master.recv_match(type='HIGHRES_IMU',blocking = True)
                writer.writerow(msg.to_dict())
            except:
                print("msg type exception")
                
def mav_print_imu():
    '''Prints IMU data from a MAVLINK message stream'''
    while True:
        try:
            msg = master.recv_match(type='HIGHRES_IMU',blocking = True)
            print(msg)
        except:
            print("msg type exception")                


# Create the connection
# Need to provide the serial port and baudrate
print("Starting application\n")
master = mavutil.mavlink_connection("COM3", baud=57600)
master.wait_heartbeat()
print('target_system {}, target component {} \n'.format(master.target_system,master.target_component))
msg = None
num_points = 1000
file = 'high_res_imu.csv'

# param_id = b'A11\0'
# master.mav.param_request_read_send(master.target_system,master.target_component, param_id, -1)

# try:
#     msg = master.recv_match(type = 'PARAM_VALUE', blocking = True)
#     print(msg)
# except:
#     print("msg type exception") 

# mav_print_imu()

# # log some data
mav_log_imu(num_points, file)

# # set up data container
# data_raw = np.zeros((num_points, 12))
# with open(file, newline='') as f:
#     reader = csv.reader(f)
#     i = 0  # row index
#     j = 0  # column index
#     # parse the values into an array
#     for row in reader:
#         j = 0
#         # skip the first row--it contains only header strings
#         if i > 0:
#             for val in row:
#                 try:
#                     num = int(val)
#                     data_raw[i-1,j] = num
#                     j = j + 1
#                 except:
#                     pass
#         i = i + 1


# # extract gyro data
# gyro_data = data_raw[:,4:7]
# i = np.arange(0,num_points)

# fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
# fig.suptitle('Gyro 180 deg Rotation Data')

# ax1.plot(i,gyro_data[:,0], label = 'x')
# ax1.set_ylabel('$\omega_x$')

# ax2.plot(i,gyro_data[:,1], label = 'y')
# ax2.set_ylabel('$\omega_y$')

# ax3.plot(i,gyro_data[:,2], label = 'z')
# ax3.set_ylabel('$\omega_z$')
# ax3.set_xlabel('time')


# # subtract bias, and scale
# gyro_bias = np.asarray([-69.342,48.32,25.984])
# dT = .02 # 50Hz data rate
# gyro_scale = 1000 *1.07 /2**16  #FS is 1000 deg/s
# gyro_data = (gyro_data - gyro_bias)*gyro_scale * dT
# np.sum(gyro_data,axis = 0)

