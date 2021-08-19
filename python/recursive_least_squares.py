import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import time, sys

from utils import Imu, CalibParams


class RecursiveLeastSquares():

	def __init__(self, lambda_, w_initial, P, del_=1, m=9):
		self.lambda_ = lambda_ #Forgetting factor
		self.m = m

		
		self.w = w_initial #Ideally from initial batch
		self.w_prev = w_initial	#For restoring params if not ellipsoid


		self.P = P #Ideally: inverse of covariance matrix

	def create_data_vector(self, vec3d, normalizer = 1.0):
		vec3d = vec3d/normalizer

		vx, vy, vz = vec3d
		vxx,vyy,vzz = vec3d*vec3d
		vxy = vx*vy
		vxz = vx*vz
		vyz = vy*vz

		x = np.array([vxx,vyy,vzz,vxy,vxz,vyz,vx, vy,vz]).reshape([1,9])

		return x

	def step(self, xi, d=1):
		"""Runs a single iteration of RLS"""
		self.w_prev = self.w

		xi = xi.reshape([self.m,1])

		#Calculate innovation and gain
		alpha_ = d - xi.T @self.w
		gain = (self.P @xi)/(self.lambda_ + xi.T @self.P @xi)

		#Update
		self.P = self.P/self.lambda_ - gain@xi.T @self.P/self.lambda_
		self.w = self.w + alpha_*gain

		return self.w

	def restore(self):
		self.w = self.w_prev
		return self.w




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
def plot_errors(errors, lambda_=0.99):
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

#------------------------------
#MAIN

def main(argv):


	#Read raw data
	filename = argv[1]
	df_raw = pd.read_csv(filename)
	normalizer =9.8
	data_raw = df_raw.values[:]/normalizer


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
	initial_batch_size = 400
	use_batch_only = False
	xi = x[:initial_batch_size,:]

	#Least squares for initial batch
	w = np.linalg.lstsq(xi, np.ones([initial_batch_size,1]), rcond=None)[0]

	#Save Initial batch results in a file
	save_batch_params = False

	if save_batch_params:
		try:
			batch_params = CalibParams.from_implicit(w)
		except:
			#If not an ellipsoid, retain bias and change A to identity
			batch_params = CalibParams.from_implicit(w, check_validity=False)
			batch_params.A = np.eye(3)

		batch_params.save('batch_params/'+argv[1].split('/')[-1][:-3]+'txt')
		print( "Saved batch parameters. Exiting...")
		exit()

	#List to save intermediate weights
	running_params = [w]*initial_batch_size

	#RLS Parameters

	m = x.shape[1]
	lambda_ = 0.99 #Forgetting factor
	del_ = 1 #Initial value of P


	#RLS Initialization

	#w = np.zeros([m,1])
	P = np.linalg.pinv(del_*np.cov(x.T))#x.T@ x#np.eye(m)
	d = 1 #Always


	index = initial_batch_size
	step = 1

	#Initializw RLS object
	rls = RecursiveLeastSquares(lambda_, w, P)


	#Estimate implicit parameters with RLS
	while index < x.shape[0] and not use_batch_only:

		xi = (x[index, :]).reshape([m,1])
		w = rls.step(xi)
		# if np.linalg.norm(xi[m-3:])<1.2:
		# 	w = rls.step(xi)
		# else:
		# 	print("outlier", index, np.linalg.norm(xi[m-3:]))

		index+=step
		running_params.append(w)

	print()
	#Implicit parameters
	print (*w)


	#Initialize variables
	last_valid_params = None
	params_i = CalibParams.from_implicit(running_params[0],check_validity=False)


	#Calculate intermaediate errors:
	running_errors = []
	for i in range(len(running_params)):
		try:
			params_i = CalibParams.from_implicit(running_params[i])
		except:
			pass
		Xcal_i = params_i.correct(data_raw[i,:])

		error = np.linalg.norm(Xcal_i)-1
		running_errors.append(error)

		last_valid_params = running_params[i] #For debugging
	

	#Get final parameters (matrix form)
	
	params = CalibParams.from_implicit(last_valid_params)#CalibParams.from_implicit(w)
	
	#Caluclate total error

	data_cal = params.correct(data_raw)

	errors = np.linalg.norm(data_cal.T,axis=1)-1
	mse = (errors@ errors.T)/data_cal.shape[1]
	
	print("\nTotal MSE = ",mse, '\n')
	print("\nA = ",params.A)
	print("\nB =", params.B)

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


	plot_errors(np.array(running_errors), rls.lambda_)

	#pd.DataFrame(data_cal.T).to_csv('tests/real/mag_calib.csv')
	#plot_errors(np.array(errors))

if __name__ == "__main__":
	main(sys.argv)