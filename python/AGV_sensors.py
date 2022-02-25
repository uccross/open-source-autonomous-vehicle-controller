# -*- coding: utf-8 -*-
"""
Created on Thu Feb 24 14:10:01 2022

@author: Aaron
"""

import pandas as pd
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

#Read raw data
# filename = sys.argv[1]
filename = 'AGV_sensor_dat.csv'

df_raw = pd.read_csv(filename)
mag_df = df_raw[['xmag','ymag']]
y_i = mag_df.values/1.0

def imu_calibrate(points, num_iters=1):
    # determine the number of points
    n = points.shape[0]
    # Implementing the algorithm from Dorveaux 2008
    # 1) initialize A_k, B_k, y_k for k=0
    A_k = np.zeros((2, 2))
    # initialize B to the mean of each dimensions--gives a more robust
    # starting estimate of the offset vector
    # B_k = np.zeros((3,1))
    B_k = -np.mean(points, 0).reshape((2, 1))
    y_k = points + B_k.T

    # initialize the estimation matrices
    A_tilde = np.eye(2)
    B_tilde = B_k

    M = np.zeros((n, 3))
    Y_k = np.zeros((n, 2))

    #Start time
    start_time = time.time()
    
    # 2) Compute (A_k+1,B_k=1) = argmin h(A,B,k) by least squares
    # batch process method
    for k in range(num_iters):
        for i in range(n):
            norm_y = np.linalg.norm(y_k[i])

            if norm_y>1.05:
                #continue
                pass
            # generate design matrix M
            M[i][0:2] = y_k[i]
            M[i][2] = 1
            # generate normalized y values, Y
            Y_k[i] = y_k[i]/norm_y

        # compute least squares estimate for matrix C such that
        # Y = M C
        C, resid, rank, sigma = np.linalg.lstsq(M, Y_k, rcond=None)
        C = C.T
        # extract the A and B matrices from C
        A_k = C[:, 0:2]
        B_k = C[:, 2].reshape((2, 1))

        # 3) update data, y_k+1
        y_k = (A_k @ y_k.T + B_k).T
        # iteratively compute A_tilde
        A_tilde = A_k @ A_tilde
        B_tilde = A_k @ B_tilde + B_k

    
    end_time = time.time()
    print("Exec time: " + str(end_time-start_time))
    return [A_tilde, B_tilde]

def plot_2d(data_raw, data_cal,title = 'Inertial Sensor Calibration Using Dorveaux'):
    # fig = plt.figure()
    # ax = fig.add_subplot()
    
    fig,ax = plt.subplots(2)
    fig.suptitle(title, fontsize=15)

    ax[0].scatter(data_raw[:, 0], data_raw[:, 1],
               s=10, marker='.', color='red', label='raw data')
    ax[1].scatter(data_cal[:, 0], data_cal[:, 1],
               s=10, marker='.', color='blue', label='calibrated data')
    
    ax[0].legend(loc="upper left")
    ax[0].set_xlabel('X')
    ax[0].set_ylabel('Y')
    ax[1].legend(loc="upper left")
    ax[1].set_xlabel('X')
    ax[1].set_ylabel('Y')
    # ax.set_title(title)
    plt.show()
    
def plot_heading(data,ref):
    fig,ax = plt.subplots(1)
    ax.plot(data, color='red', label='mag')
    ax.plot(ref, color = 'blue', label='GPS cog')
    ax.legend()
    ax.set_xlabel('Index')
    ax.set_ylabel('Heading [deg]')
    fig.suptitle('Calibrated Magnetic Heading vs GPS Course Over Ground', fontsize=15)
    fig.show()    
    
    
def calc_heading(data, offset =12.98):
    n= data.shape[0]
    heading = np.zeros((n,1))
  # heading in degrees:
    heading = np.degrees(np.arctan2(data[:,1],data[:,0]))
  # add magnetic declination to measurement 
    for i in range(n):
        heading[i] = heading[i] + offset
        if heading[i] > 180:
            heading[i] = heading[i] - 180
    
    return heading
        
    
[A_cal, B_cal] = imu_calibrate(y_i, 10)
data_cal = ((A_cal @ y_i.T) + B_cal).T

plot_2d(y_i, data_cal, 'Magnetometer 2D Calibration')
heading = calc_heading(data_cal)
ref = df_raw['cog'].values / 100
plot_heading(heading + 180, ref)


