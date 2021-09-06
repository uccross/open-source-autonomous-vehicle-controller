# -*- coding: utf-8 -*-
import time
from pymavlink import mavutil
import os
import csv


# Create the connection
# Need to provide the serial port and baudrate
print("Starting mavcsv_logging.py\n")
# master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600) # usb on Pi
master = mavutil.mavlink_connection("COM6", baud=115200) # usb on Pi
# find the OSAVC controller
master.wait_heartbeat(timeout = 10)
if master.target_system == 0:
    print('No system detected!')
else:
    print('target_system {}, target component {} \n'.format(master.target_system,master.target_component))

# setup automatic text logging.  NB: can't read these files yet!  TODO:  email forum for help
# file = '/mnt/usb/logfiles/logfile.log'
file = './logfile.log'
# initiate logging
master.setup_logfile(file)

# but we're going to make our own datalogging routine using CSV module
csv_file = './logfile.csv'

# first find all the incoming messages:
msgs_dict = {}
# Collect messages for one second to get all keys
start_time = time.time()
wait_time = 5
print("Collecting messages for {} seconds to get all keys".format(wait_time))
while time.time() - start_time < wait_time:
    try:
        msg = master.recv_match(blocking = True)
        # add msg to the msgs_dict
        msgs_dict.update(msg.to_dict())
    except:
        print('msg error, or dict error!')
        
print("Finished collecting messages")

# Put all  keys for all the incoming messages into the headers list 
headers = list(msgs_dict)
print(headers)

with open(csv_file, 'w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=headers)    
    writer.writeheader()
    start_time = time.time()
    logging_time = 30
    # currently we log for a specified period of time
    while time.time() - start_time < logging_time:
        try:
            msg = master.recv_match(blocking = True)

            print(type(msg))
            print(msg)

            # add msg to the msgs_dict
            msgs_dict.update(msg.to_dict())
            # and write it to the file
            writer.writerow(msgs_dict)

            # echo_msg = master.

        except:
            time.sleep(1)
            print('msg error, or dict error!')

# finish up:
master.close()
print('Exiting datalogging script')



