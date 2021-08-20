import numpy as np
import pandas as pd
import datetime


#x = AX+B

#Calib matrix
# scales = [0.984, 1.07, 0.995]
# rot = [[0.9716406,  0.1448620,  0.1868945],\
# 		[-0.1410913,  0.9894332, -0.0333945],\
# 		[-0.1897572,  0.0060782,  0.9818122]] #EulerXYZ: 0.034, 0.188, -0.148
# skews = np.eye(3)

# A = np.matmul(rot, np.diag(scales))
# A = np.matmul(skews, A)
A = np.array([[1.3, 0.03, 0.04],[0.026, 1.56, 0.0],[0.011, 0.12, 0.8]])

print(A)

#Bias
B = np.array([[-0.0568], [0.055], [0.109]])
print(B)

#Variance
var = 0.1

#Generate points
num_points = 2_000
r =1

points = []
for i in range(num_points):
	theta = np.random.rand()*2*np.pi
	phi = np.random.rand()*2*np.pi
	x = r*np.sin(phi)*np.cos(theta)
	y = r*np.sin(phi)*np.sin(theta)
	z = r*np.cos(phi)
	points.append([x,y,z])

df_calib = pd.DataFrame(points)


#Simulate Drift (1. sudden-drift)
simulate_drift = False

if simulate_drift:
	scale_drift = np.diag(np.random.randn(3)*0.1 +1)
	rot_drift = np.random.randn(3,3)*0.002
	z = (scale_drift==0)*rot_drift + scale_drift
	bias_drift = np.random.randn(3,1)*0.2

	#A = z@A
	B = B+bias_drift
	print('\n Drifted params:\n')
	print(A)
	print(B)

#Corrupt
points_raw = []
for p in points:
	p_ = np.matmul(A,p)+B.T
	p_ += np.random.randn(3)*var
	points_raw.append(*(p_.tolist()))

assert(len(points_raw)==len(points))

df_raw = pd.DataFrame(points_raw)


#Save CSVs
stamp = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')

df_calib.to_csv('points_calib__'+stamp+'.csv', header=False, index=False)
df_raw.to_csv('points_raw__'+stamp+'.csv', header=False, index=False)

