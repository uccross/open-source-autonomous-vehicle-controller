"""
.. module:: plot_mav_csv.py
    :platform: MacOS, Unix, Windows,
    :synopsis: Plot data in csv's generated by mavcsv_logging.py. Specify the 
    MAVLink message type, and parameters by specifying column names (or header)
    arguments. For example:

    python plot_mav_csv.py ../../my_data/2021_08_12.csv GPS_RAW_INT lat lon

    where '../../my_data/2021_08_12.csv' is the file path and name, 
    'GPS_RAW_INT' is the MAVLink message type,
    'lat' is a parameter within that message type we want to plot,
    'lon' is a parameter also within that message type we want to plot.

.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from mpl_toolkits import mplot3d

import argparse

###############################################################################
# Parse Arguments
parser = argparse.ArgumentParser()

parser.add_argument('csv_file_path',
                    nargs='?',
                    default='<enter_file_path>.csv',
                    help='A valid path to a MAVLink csv file generated by \
                        mavcsv_logging.py')

parser.add_argument('mav_message',
                    nargs='?',
                    default='GPS_RAW_INT',
                    help='A valid MAVLink message name, for example: \
                        GPS_RAW_INT. For more examples see \
                            https://mavlink.io/en/messages/common.html')

parser.add_argument('column_header_0',
                    nargs='?',
                    default='lat',
                    help='A valid column based on one of the csv header names,\
                         for example \'lat\'')

parser.add_argument('column_header_1',
                    nargs='?',
                    default='lon',
                    help='A valid column based on one of the csv header names,\
                         for example \'lon\'')

parser.add_argument('column_header_2',
                    nargs='?',
                    default='alt',
                    help='A valid column based on one of the csv header names,\
                         for example \'alt\'')

arguments = parser.parse_args()

csv_file_path = arguments.csv_file_path
mav_message = arguments.mav_message
column_header_0 = arguments.column_header_0
column_header_1 = arguments.column_header_1
column_header_2 = arguments.column_header_2

data_start_index = 17000  # @TODO: make this a mandatory argument

# -1 because we don't count the scpit name itself.
num_args = len(sys.argv) - 1
num_cols = num_args - 1  # subtract 1 again so as not to count the csv file

print('Number of arguments given: {}'.format(num_args))
print('\r\nMAVLink message type to plot: {}'.format(mav_message))

columns = sys.argv[-num_cols:]

print("\r\nUsing {} csv names (headers): ".format(num_cols))
for col in columns:
    print('    {}'.format(col))

mav_params = columns[1:]
print("\r\nMAVLink parameter column names (headers): ")
for col in mav_params:
    print('    {}'.format(col))

###############################################################################
# Open the MAVLink csv file
# Data is initially a 1x(n-1) because the first column holds the MAVLink
# message names along with the other specified csv header column names of
# interest: column_header_{0}, column_header_{1}, ... column_header_{n-1}
#
# We loop through the MAVLink messages of the csv file, and if we find one of
# the target mavlink messages, such as GPS_RAW_INT, then we get the data in
# that messages specified by the column name (or header) arguments.

n_data = num_cols-1  # -1 so we don't inclue the MAVLink message type
target_mav_data = np.zeros((1, n_data))

# try:
print('\r\ncsv_file_path = {}'.format(csv_file_path))
print('type = {}'.format(type(csv_file_path)))

with open(csv_file_path, 'r') as read_obj:
    csv_reader = csv.reader(read_obj)
    csv_dict_reader = csv.DictReader(read_obj)
    column_names = csv_dict_reader.fieldnames

    print('\r\nAll column names: {}'.format(column_names))

    skip_index = 0

    for row in csv_dict_reader:
        if ((row[column_names[0]] == mav_message) and
                (skip_index >= data_start_index)):

            # @NOTE: these are strings in a matrix at this point, NOT floats,
            # or ints.
            new_data = np.zeros((1, n_data))
            for param, i in zip(mav_params, range(n_data)):
                new_data[0, i] = float(row[param])  # Now we have a non-string

                ###############################################################
                # Rules for specific MAVLink messages
                ###############################################################
                if mav_message == 'GPS_RAW_INT':

                    if (param == 'lat') or (param == 'lon'):
                        new_data[0, i] /= 10000000.0

                    if (param == 'current_distance'):
                        new_data[0, i] /= -1000.0

            ###################################################################
            target_mav_data = np.concatenate((target_mav_data, new_data),
                                             axis=0)

        skip_index += 1

# except:
#     print('Enter a valid MAVLink csv file path.')
#     sys.exit()

###############################################################################
# Find and throw away outliers, for example a row of np.array([lat: 0, lon: 0])
# for data in 'GPS_RAW_INT' is probably not correct if we were testing GPS
# in Santa Cruz California, USA
target_mav_data_t = np.transpose(target_mav_data)
sigma_vec = np.zeros((n_data, 1))
mu_vec = np.zeros((n_data, 1))

# Use the transpose of the target mavlink data temporarily for convenience
for row, sigma, i in zip(target_mav_data_t, sigma_vec, range(n_data)):
    mu_vec[i, 0] = np.mean(row)
    sigma = np.std(row)
    sigma_vec[i, 0] = sigma
    print('MAVLink parameter {}: {} standard deviation: {}'.format(
        i,
        sys.argv[-n_data+i],
        sigma))

for row, h in zip(target_mav_data, range(len(target_mav_data))):
    for col, i in zip(row, range(n_data)):

        if np.abs(col - mu_vec[i, 0]) > sigma_vec[i, 0]*2.0:
            target_mav_data = np.delete(target_mav_data, h, 0)
            print('deleting row {} because column {} holds an outlier'.format(
                h, i))

print('target_mav_data: \r\n{}'.format(target_mav_data))
###############################################################################
# Plot

margins = 0.0001

fig = plt.figure()
ax = plt.gca()

# If there are 3 MAVLink parameter arguments for this script, then assume 3D
# plot. @TODO: Find a more elegant, all-encompassing way to do this
if len(mav_params) == 3:
    ax = plt.axes(projection='3d')
    # ax = fig.add_subplot(projection='3d')
    ax.plot(target_mav_data[:, 0],
               target_mav_data[:, 1],
               target_mav_data[:, 2])
    # ax.set_zlim(min(target_mav_data[:, 2]) - margins,
    #             max(target_mav_data[:, 2]) + margins)
    ax.set_zlabel('{}'.format(mav_params[2]))
else:
    plt.scatter(target_mav_data[:, 0], target_mav_data[:, 1])

plt.xlabel('{}'.format(mav_params[0]))
plt.ylabel('{}'.format(mav_params[1]))

ax.set_xlim(min(target_mav_data[:, 0]) - margins,
            max(target_mav_data[:, 0]) + margins)
ax.set_ylim(min(target_mav_data[:, 1]) - margins,
            max(target_mav_data[:, 1]) + margins)

plt.title('{}'.format(mav_message))
# ax.ticklabel_format(useOffset=False)

# precision = 4
# ax.yaxis.set_major_formatter(FormatStrFormatter('%.{}f'.format(precision)))
# ax.xaxis.set_major_formatter(FormatStrFormatter('%.{}f'.format(precision)))
# step_size_x = (max(target_mav_data[:, 0]) - min(target_mav_data[:, 0])) / 8.0
# step_size_y = (max(target_mav_data[:, 1]) - min(target_mav_data[:, 1])) / 8.0
# plt.xticks(np.arange(min(target_mav_data[:, 0]),
#                      max(target_mav_data[:, 0]),
#                      step_size_x))
# plt.yticks(np.arange(min(target_mav_data[:, 1]),
#                      max(target_mav_data[:, 1]),
#                      step_size_y))
plt.grid()
plt.show()
