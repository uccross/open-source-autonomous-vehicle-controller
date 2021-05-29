# -*- coding: utf-8 -*-
"""
Created on Thu May 27 10:53:42 2021

@author: Aaron
"""

import matplotlib.pyplot as plt
import numpy as np

def generate_sphere(n,center = np.array([0.,0.,0.]), radius = 1.0):
        # generate the random quantities
    phi = np.random.uniform( 0, 2*np.pi, n)
    theta = np.random.uniform(0,np.pi,n)
    
    points = np.zeros((n,3))
    
    for i in range(n):
        points[i] = center + [radius*np.cos(phi[i])*np.sin(theta[i]),radius*np.sin(phi[i])*np.sin(theta[i]),radius*np.cos(theta[i])]
        
    return points

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

#number of datapoints to generate
n = 1000
radius = 1.0
center = np.array([0,0,0])
#noise parameter
sigma = 0.1

#get the points
points = generate_sphere(n, center, radius)

#make some noise
noise =sigma * np.random.randn(n,3)
#add noise to the data
points = points + noise
ax.scatter(points[:,0],points[:,1],points[:,2],s = 10,marker ='.',color = 'green')

#add some scaling, misalignment
A = np.array([[1.7, 1, 1],[0, 1.3, 1],[0, 0, 1.9]])
#offsets
B = np.array([[0.3],[0.3],[0.5]])

#corrupt the points
points = (A @ points.T + B).T
ax.scatter(points[:,0],points[:,1],points[:,2],s = 10,marker ='.',color = 'red')


#Implement the Dorveaux algorithm
#1) initialize k=0, y_k = y_i for all points, i
num_iters = 1
A_k = np.zeros((3,3))
B_k = np.zeros((3,1))
y_k = points

#initialize the estimation matrices
A_tilde = np.eye(3)
B_tilde = B_k

#2) Compute (A_k+1,B_k=1) = argmin h(A,B,k) by least squares

#batch process attempt
M = np.zeros((n,4))
Y_k = np.zeros((n,3))

for k in range(num_iters):
    for i in range(n):
        norm_y = np.linalg.norm(y_k[i])
        M[i][0:3] = y_k[i]
        M[i][3] = 1
        Y_k[i] = y_k[i]/norm_y
        
    C, resid, rank, sigma = np.linalg.lstsq(M,Y_k,rcond=None)
    C = C.T
    A_k = C[:,0:3]
    B_k = C[:,3].reshape((3,1))

    
    # 3) update data, y_k+1
    y_k = (A_k @ y_k.T  + B_k).T
    ax.scatter(y_k[:,0],y_k[:,1],y_k[:,2],s = 10,marker ='.')
    
    # iteratively compute A_tilde
    A_tilde = A_k @ A_tilde
    B_tilde = A_k @ B_tilde + B_k


# #element-wise
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
    
#     #3) update data, y_k+1
#     y_k = (A_k_e @ y_k.T  + B_k_e).T
#     # ax.scatter(y_k[:,0],y_k[:,1],y_k[:,2],s = 10,marker ='.')
    
#     # iteratively compute A_tilde
#     A_tilde_e = A_k_e @ A_tilde_e
#     B_tilde_e = A_k_e @ B_tilde_e + B_k_e

    
#test solution
y = np.matmul(A_tilde,points.T).T + B_tilde.T
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(points[:,0],points[:,1],points[:,2],s = 10,marker ='.',color = 'red', alpha = 0.5, label = 'raw data')
ax.scatter(y[:,0],y[:,1],y[:,2],s = 10,marker ='.',color = 'blue', label = 'calibrated')
ax.legend(loc = "upper left")
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Magnetometer Calibration using Dorveaux Algorithm')



