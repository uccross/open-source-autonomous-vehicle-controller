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


#RLS Parameters

m = x.shape[1]
lambda_ = 0.99 #Forgetting factor
del_ = 1e5 #Initial value of P


#Initialization

w = np.random.rand(m,1)	#Weights [A; b]
P = x.T@x#del_*np.eye(m)
xi = x[0:,:]
d = 1 #Always


index = 0
step = 1

while index < x.shape[0]:

	xi = (x[index, :]).reshape([m,1])
	alpha_ = d - xi.T@w
	gain = (P @xi)/(lambda_ + xi.T @P @xi)
	print("Innovation: ",alpha_)

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
rearrange = np.array([[0,0,1], [1,0,0],[0,1,0]]) #Temporary workaround
R = -eig_vecs @ rearrange
scales = np.sqrt(-Q2[3,3]/eig_vals) @ rearrange

print("\nRot: ",R)
print("\nBias ", B)
print("\nScales ", scales)