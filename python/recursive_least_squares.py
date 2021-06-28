import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import time, sys

#Read raw data
filename = sys.argv[1]
df_raw = pd.read_csv(filename)
data_raw = df_raw.values[:]

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
initial_batch_size = 200
use_batch_only = False
xi = x[:initial_batch_size,:]

#Least squares for initial batch
w = np.linalg.lstsq(xi, np.ones([initial_batch_size,1]), rcond=None)[0] 

#List to save intermediate weights
running_params = [w]*initial_batch_size

#RLS Parameters

m = x.shape[1]
lambda_ = 0.99 #Forgetting factor
del_ = 1 #Initial value of P


#RLS Initialization

#w = np.random.rand(m,1)
P = np.linalg.pinv(del_*np.cov(x.T))#x.T@ x#np.eye(m)
d = 1 #Always


index = initial_batch_size
step = 1

#Estimate implicit parameters with RLS
while index < x.shape[0] and not use_batch_only:

	xi = (x[index, :]).reshape([m,1])
	alpha_ = d - xi.T@w
	gain = (P @xi)/(lambda_ + xi.T @P @xi)
	#print("Innovation: ",alpha_)

	#Update
	P = P/lambda_ - gain@xi.T @P/lambda_
	w = w + alpha_*gain

	index+=step
	running_params.append(w)

print()
#Implicit parameters
print (*w)


def recover_params(w, verbose=True):
	"""
	Converts implicit linear form of the ellipse to 
	the matrix form (AX+B)@(AX+B).T = 1

	Arguments:
		w: Implicit parameters
	Returns:
		A: Scale, Rotation & Skew matrix
		B: Bias

	"""
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
	A = R@ np.diag(scales) @R.T

	if verbose:
		print("\nRot: ",R)
		print("\nCross matrix: ", R.T@R)
		print("\nScales ", scales)
		print("\n---\nA = ", A)
		print("\nBias = ", B)

	return A, B



#Plot error
def plot_errors(errors):
	"""
	Plots a moving average and histogram of np array errors
	"""
	num_vals = int(errors.shape[0]/50)
	error_avg = [np.mean((errors*errors)[i-num_vals:i]) for i in range(num_vals,errors.shape[0]-2)]
	
	fig,ax = plt.subplots(2)
	fig.suptitle("RLS with forgetting factor="+str(lambda_), fontsize=15)

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


#Get final parameters (matrix form)
A,B = recover_params(w)


#Calculate intermaediate errors:
running_errors = []
for i in range(len(running_params)):
	Ai, Bi = recover_params(running_params[i], verbose=False)
	Xcal_i = np.linalg.inv(Ai)@(data_raw[i,:]-Bi)
	error = np.linalg.norm(Xcal_i)-1
	running_errors.append(error)

#Caluclate total error

data_cal = np.linalg.inv(A)@(data_raw - B).T
errors = np.linalg.norm(data_cal.T,axis=1)-1
mse = (errors@ errors.T)/data_cal.shape[1]
print("\nTotal MSE = ",mse, '\n')
print("\nA = ",np.linalg.inv(np.linalg.pinv(A)))
print("\nB =", B)

#For drift/parameter change:
switch_index = int(data_cal.shape[1]/2)
errors1 = np.array(running_errors[initial_batch_size+1:switch_index-1]) #Before param change
errors2 = np.array(running_errors[switch_index+400:]) #After drift + 400 measurements
mse1 = (errors1@errors1.T)/errors1.shape[0]
mse2 = (errors2@errors2.T)/errors2.shape[0]
std2 = np.std(errors2)
print("\nMSE (before) = ", mse1)
print("\nMSE (after) = ", mse2 )
print("\nStd dev. after = ", std2)
#print(errors1.shape,errors2.shape)


plot_errors(np.array(running_errors))

#plot_errors(np.array(errors))
