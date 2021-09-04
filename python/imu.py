# -*- coding: utf-8 -*-
"""
Created on Wed Jun  2 07:57:19 2021

@author: Aaron
"""

import matplotlib.pyplot as plt
import numpy as np
import csv


def generate_sphere(n, center=np.array([0., 0., 0.]), radius=1.0):
        # generate the random quantities
    phi = np.random.uniform(0, 2*np.pi, n)
    theta = np.random.uniform(0, np.pi, n)

    points = np.zeros((n, 3))

    for i in range(n):
        points[i] = center + [radius*np.cos(phi[i])*np.sin(theta[i]), radius*np.sin(
            phi[i])*np.sin(theta[i]), radius*np.cos(theta[i])]

    return points


def sim_raw_data(n=1000):
    # number of datapoints to generate
    radius = 1.0
    center = np.array([0, 0, 0])
    # noise parameter
    sigma = 0.1

    # get the uniformly distributed points on unit sphere
    points = generate_sphere(n, center, radius)

    # make some noise
    noise = sigma * np.random.randn(n, 3)
    # add noise to the data
    points = points + noise

    # generate the A matrix--
    # values on the diagonal are scaling errors
    # off diagonal elements are misalignents
    A = np.array([[1.7, 1, 1], [0, 1.3, 1], [0, 0, 1.9]])
    # B matrix (vector) moves the points away from the origin
    # therefore are a bias vector
    B = np.array([[.25], [0.5], [-0.5]])

    # corrupt the points
    points = (A @ points.T + B).T
    return points


def imu_calibrate(points, num_iters=1):
    # determine the number of points
    n = points.shape[0]
    # Implementing the algorithm from Dorveaux 2008
    # 1) initialize A_k, B_k, y_k for k=0
    A_k = np.zeros((3, 3))
    # initialize B to the mean of each dimensions--gives a more robust
    # starting estimate of the offset vector
    # B_k = np.zeros((3,1))
    B_k = -np.mean(points, 0).reshape((3, 1))
    y_k = points + B_k.T

    # initialize the estimation matrices
    A_tilde = np.eye(3)
    B_tilde = B_k

    # 2) Compute (A_k+1,B_k=1) = argmin h(A,B,k) by least squares
    # batch process method
    M = np.zeros((n, 4))
    Y_k = np.zeros((n, 3))

    for k in range(num_iters):
        for i in range(n):
            norm_y = np.linalg.norm(y_k[i])
            # generate design matrix M
            M[i][0:3] = y_k[i]
            M[i][3] = 1
            # generate normalized y values, Y
            Y_k[i] = y_k[i]/norm_y

        # compute least squares estimate for matrix C such that
        # Y = M C
        C, resid, rank, sigma = np.linalg.lstsq(M, Y_k, rcond=None)
        C = C.T
        # extract the A and B matrices from C
        A_k = C[:, 0:3]
        B_k = C[:, 3].reshape((3, 1))

        # 3) update data, y_k+1
        y_k = (A_k @ y_k.T + B_k).T
        # ax.scatter(y_k[:,0],y_k[:,1],y_k[:,2],s = 10,marker ='.')

        # iteratively compute A_tilde
        A_tilde = A_k @ A_tilde
        B_tilde = A_k @ B_tilde + B_k

    # # Alternatively we can compute it element-wise
    # A_k_e = np.zeros((3,3))
    # B_k_e = np.zeros((3,1))
    # y_k = points
    # M = np.zeros((3*n,12))
    # Y_k = np.zeros((3*n))

    # #initialize the estimation matrices
    # A_tilde_e = np.eye(3)
    # B_tilde_e = B_k_e

    # for k in range(num_iters):
    #     for i in range(n):
    #         norm_y = 1/ np.linalg.norm(y_k[i])
    #         M[3*i,0:3] =y_k[i]
    #         M[3*i,9] = 1
    #         M[3*i +1,3:6] =y_k[i]
    #         M[3*i+1,10] = 1
    #         M[3*i+2,6:9] =y_k[i]
    #         M[3*i+2,11] = 1
    #         Y_k[3*i] = y_k[i,0]*norm_y
    #         Y_k[3*i+1] = y_k[i,1]*norm_y
    #         Y_k[3*i+2] = y_k[i,2]*norm_y

    #     C, resid, rank, sigma = np.linalg.lstsq(M,Y_k,rcond=None)
    #     D = C.reshape((4,3))
    #     A_k_e = np.array([[C[0],C[1],C[2]], [C[3],C[4],C[5]], [C[6],C[7],C[8]]])
    #     B_k_e = C[9:12].reshape((3,1))

    return [A_tilde, B_tilde]

def plot_3d(data_raw, data_cal,title = 'Inertial Sensor Calibration Using Dorveaux'):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(data_raw[:, 0], data_raw[:, 1], data_raw[:, 2],
               s=10, marker='.', color='red', label='raw data')
    ax.scatter(data_cal[:, 0], data_cal[:, 1], data_cal[:, 2],
               s=10, marker='.', color='blue', label='calibrated data')
    ax.legend(loc="upper left")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    
# test simulated data
num_points = 2000
data_raw = sim_raw_data(num_points)
[A_cal, B_cal] = imu_calibrate(data_raw, 20)
data_cal = ((A_cal @ data_raw.T) + B_cal).T

plot_3d(data_raw,data_cal)


# test the accelerometer data
data_raw = np.zeros((num_points, 12))
with open('imu_tmbl_070121.csv', newline='') as f:
    reader = csv.reader(f)
    i = 0  # row index
    j = 0  # column index
    # parse the values into an array
    for row in reader:
        j = 0
        # skip the first row--it contains only header strings
        if i > 0:
            for val in row:
                try:
                    num = int(val)
                    data_raw[i-1,j] = num
                    j = j + 1
                except:
                    pass
        i = i + 1
mag_field_strength = 47.498 #micro-tesla
bits_2_g = 1/2**14  
bits_2_T = (4900/2**15) / mag_field_strength          
accel_data = data_raw[:,1:4] * bits_2_g
mag_data = data_raw[:,7:10] * bits_2_T
[A_acc, B_acc] = imu_calibrate(accel_data, 20)
[A_mag, B_mag] = imu_calibrate(mag_data, 20)
accel_data_cald = (A_acc @ accel_data.T + B_acc).T
mag_data_cald = (A_mag @ mag_data.T + B_mag).T

plot_3d(accel_data,accel_data_cald,'Accelerometer Sensor Calibration Using Dorveaux')
plot_3d(mag_data,mag_data_cald,'Magnetometer Sensor Calibration Using Dorveaux')



err_raw = np.linalg.norm(accel_data, axis = 1) -1
err_cal = np.linalg.norm(accel_data_cald, axis = 1) -1
i = np.arange(0,num_points)
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.plot(i,err_raw, color = 'red', label = 'raw')
ax.plot(i,err_cal, color = 'blue', label = 'cal', alpha = 0.5)

err_raw = np.linalg.norm(mag_data, axis = 1) -1
err_cal = np.linalg.norm(mag_data_cald, axis = 1) -1
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.plot(i,err_raw, color = 'red', label = 'raw')
ax.plot(i,err_cal, color = 'blue', label = 'cal', alpha = 0.5)
ax.set_title('Magnetometer Error')
ax.set_xlabel('Data Index')
ax.set_ylabel('Error [au]')
ax.legend(loc= "upper right")

