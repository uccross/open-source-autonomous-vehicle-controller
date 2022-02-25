import pandas as pd
import sys
import time
import numpy as np
import matplotlib.pyplot as plt


#Read raw data
# filename = sys.argv[1]
filename = 'magcal_2D.csv'

df_raw = pd.read_csv(filename)

data_raw = df_raw.values[:,:]/0.43

#Dorveaux calibration (copy pasted from Aaron's code)

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


#Plot error
def plot_errors(errors):
    """
    Plots a moving average and histogram of np array errors
    """
    num_vals = int(errors.shape[0]/50)
    error_avg = [np.mean((errors*errors)[i-num_vals:i]) for i in range(num_vals,errors.shape[0]-2)]
    
    fig,ax = plt.subplots(2)
    fig.suptitle("Dorveaux", fontsize=15)

    #Running error
    ax[0].plot([i for i in range(num_vals, errors.shape[0]-2)], error_avg)
    ax[0].set_title("Squared error vs iteration")

    #Histogram
    ax[1].set_title("Error Histogram")
    n, bins, __ = ax[1].hist(errors, 50)

    sigma = np.std(errors)
    mu = np.mean(errors)
    y = ((1/(np.sqrt(2*np.pi)*sigma))*np.exp(-0.5*(1/sigma*(bins-mu))**2))*len(errors)*(bins[2]-bins[1])
    print("Mean error: ", mu)
    print("Sigma: ",sigma)
  
    ax[1].plot(bins, y, '--', color ='black')
    
    plt.show()

[A_cal, B_cal] = imu_calibrate(data_raw, 10)
data_cal = ((A_cal @ data_raw.T) + B_cal).T

plot_2d(data_raw, data_cal, 'Magnetometer 2D Calibration')

#plot_3d(data_raw,data_cal, "Simulated Data")

errors = np.linalg.norm(data_cal,axis=1)-1
print(errors)
mse = (errors@ errors.T)/data_cal.shape[0]


#Plot/Show
print("\nA = ", np.linalg.inv(A_cal))
print("\nB = ", -B_cal)




