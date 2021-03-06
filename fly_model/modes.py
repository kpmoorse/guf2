import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def npread(file, sep=',', header=None):
    df = pd.read_csv(file, sep=sep, header=header)
    arr = np.asarray(df)
    return arr

def mix(color1, color2, val):
	res = []
	for c1, c2 in zip(color1,color2):
		res.append(c1*(1-val)+c2*(val))
	return res

def read_modes():

	a = {}
	X = {}
	b = {}

	for angle in ["eta", "phi", "theta"]:

		# Import hover flight coefficients
		a[angle] = npread('legendre/hover/a_{}.csv'.format(angle))

		# Import conversion matrices
		X[angle] = npread('legendre/hover/X_{}.csv'.format(angle))

		b[angle] = {}
		for wing in ["L","R"]:

			# Import control mode coefficients
			# 	F = Force, M = Torque
			# 	x,y,z = axes
			# 	P = positive, N = negative

			b[angle][wing] = {

				# Forces
				"FxN": -npread('legendre/forces/b_FxB_{}.csv'.format(angle)),
				"FxP": npread('legendre/forces/b_FxF_{}.csv'.format(angle)),
				"FyN": -npread('legendre/forces/b_FyR_{}{}.csv'.format(angle, wing)),
				"FyP": -npread('legendre/forces/b_FyL_{}{}.csv'.format(angle, wing)),
				"FzN": npread('legendre/forces/b_FzD_{}.csv'.format(angle)),
				"FzP": -npread('legendre/forces/b_FzU_{}.csv'.format(angle)),

				# "FxN": -npread('legendre/forces/b_FxF_{}.csv'.format(angle)),
				# "FxP": npread('legendre/forces/b_FxF_{}.csv'.format(angle)),
				# "FyN": npread('legendre/forces/b_FyL_{}{}.csv'.format(angle, wing)),
				# "FyP": -npread('legendre/forces/b_FyL_{}{}.csv'.format(angle, wing)),
				# "FzN": npread('legendre/forces/b_FzU_{}.csv'.format(angle)),
				# "FzP": -npread('legendre/forces/b_FzU_{}.csv'.format(angle)),

				# Torques
				"MxN": npread('legendre/torques/b_MxL_{}{}.csv'.format(angle, wing)),
				"MxP": npread('legendre/torques/b_MxR_{}{}.csv'.format(angle, wing)),
				"MyN": npread('legendre/torques/b_MyU_{}.csv'.format(angle)),
				"MyP": -npread('legendre/torques/b_MyD_{}.csv'.format(angle)),
				"MzN": -npread('legendre/torques/b_MzL_{}{}.csv'.format(angle, wing)),
				"MzP": -npread('legendre/torques/b_MzR_{}{}.csv'.format(angle, wing))

				# "MxN": -npread('legendre/torques/b_MxR_{}{}.csv'.format(angle, wing)),
				# "MxP": npread('legendre/torques/b_MxR_{}{}.csv'.format(angle, wing)),
				# "MyN": -npread('legendre/torques/b_MyD_{}.csv'.format(angle)),
				# "MyP": npread('legendre/torques/b_MyD_{}.csv'.format(angle)),
				# "MzN": -npread('legendre/torques/b_MzL_{}{}.csv'.format(angle, wing)),
				# "MzP": npread('legendre/torques/b_MzL_{}{}.csv'.format(angle, wing))

			}

	return a, X, b

def calc_kinematics(a, X, b, cmd):

	# cmd contains the mode strengths (Fx, Fy, Fz, Mx, My, Mz)
	# result contains the kinematics (etaL, phiL, thetaL, etaR, phiR, thetaR)

	modes = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
	# modes = ["Fz"]
	kinematics = np.empty((100,0))

	for wing in ["L", "R"]:

		for angle in ["phi", "theta", "eta"]:

			# Initialize legendre coefficients
			legendre = np.copy(a[angle])
			
			# Augment coefficients for each nonzero mode
			for i, mode in enumerate(modes):

				if cmd[i] == 0:
					continue

				sign = ("N","P")[int(cmd[i]>0)]
				# print(mode+sign)
				legendre += abs(cmd[i])*b[angle][wing][mode+sign]

			# Convert legendre coefficients to time series
			kinematics = np.hstack((kinematics,(X[angle].dot(legendre))))
	
	return kinematics

if __name__ == '__main__':

	a, X, b = read_modes()
	# legendre = init_legendre()

	angles = ["phi", "theta", "eta"]
	c = plt.rcParams['axes.prop_cycle'].by_key()['color']
	print(c[0])

	for i, x in enumerate(np.arange(0,6e-5,1e-5)):
		for j in range(3):
			plt.subplot(211)
			# plt.plot(calc_legendre(legendre, x)[:,j], color=c[j])
			plt.subplot(212)
			plt.plot(calc_kinematics(a,X,b,[0,0,0,0,0,x])[:,j], color=c[j])

	plt.show()