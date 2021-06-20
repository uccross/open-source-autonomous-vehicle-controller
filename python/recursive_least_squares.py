import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import time, sys

#Read raw data
filename = sys.argv[1]
df_raw = pd.read_csv(filename)
data_raw = df_raw.values

#Create data vector
vx = data_raw[:,0].reshape(data_raw.shape[0],1)
vy = data_raw[:,1].reshape(data_raw.shape[0],1)
vz = data_raw[:,2].reshape(data_raw.shape[0],1)
vxx = vx*vx
vyy = vy*vy
vzz = vz*vz
vxy = vx*vy
vxz = vx*vz
vyz = vy*vz
#vc = np.ones(vx.shape)

x = np.concatenate((vxx,vyy,vzz,vxy,vxz,vyz,vx, vy,vz), axis=1)
print(x.shape)

"""
Measurement/Observation: IMU readings (x)
Estimater: Ellipsoid parameters (w)

Measurement model: x.T@w = 1,
where x is in the implicit form (9x1)


"""
#Batch calibration parameters
initial_batch_size = 999
xi = x[:initial_batch_size,:]

#Least squares for initial batch
w = np.linalg.lstsq(xi, np.ones([initial_batch_size,1]), rcond=None)[0] 


#RLS Parameters

m = x.shape[1]
lambda_ = 0.99 #Forgetting factor
del_ = 1 #Initial value of P


#RLS Initialization

#w = np.random.rand(m,1)
P = del_*np.cov(x.T)#x.T@ x#np.eye(m)
d = 1 #Always


index = initial_batch_size
step = 1

#Estimate implicit parameters with RLS
while index < x.shape[0]:

	xi = (x[index, :]).reshape([m,1])
	alpha_ = d - xi.T@w
	gain = (P @xi)/(lambda_ + xi.T @P @xi)
	#print("Innovation: ",alpha_)

	#Update
	P = P/lambda_ - gain@xi.T @P/lambda_
	w = w + alpha_*gain

	index+=step


print()
#Implicit parameters
print (*w)

w = w.reshape([9,])

#Matrix form of ellipse
Q = np.array([[w[0], w[3]/2, w[4]/2, w[6]/2],\
				[w[3]/2, w[1], w[5]/2, w[7]/2],\
				[w[4]/2, w[5]/2, w[2], w[8]/2],\
				[w[6]/2, w[7]/2, w[8]/2, -1.0]])

#Recover bias
B = np.linalg.lstsq(Q[:3,:3], -Q[:3,3], rcond=None)[0]

#Translate
T = np.eye(4)
T[:3,3] = B
Q2 = T.T @Q @T

#Find eigenvalues
eig_vals, eig_vecs = np.linalg.eig(Q2[:3,:3])

#Recover Rotation and Scales
rearrange = np.array([[0,0,1], [1,0,0],[0,1,0]]) #Temporary workaround for axis rearrange
R = -eig_vecs @ rearrange
scales = np.sqrt(-Q2[3,3]/eig_vals) @ rearrange

#Recover A
A = R@ np.diag(scales)

print("\nRot: ",R)
print("\nScales ", scales)
print("\nA = ", A)
print("\nBias = ", B)


#Caluclate error

data_cal = np.linalg.inv(A)@(data_raw - B).T
errors = np.linalg.norm(data_cal.T,axis=1)-1
mse = (errors@ errors.T)/data_cal.shape[1]
print("\nMSE = ",mse, '\n')


#Plot error
num_vals = int(errors.shape[0]/25)
error_avg = [np.mean((errors*errors)[i:i+num_vals]) for i in range(num_vals,errors.shape[0]-2)]
plt.plot([i for i in range(num_vals, errors.shape[0]-2)], error_avg)
#plt.show()