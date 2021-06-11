import flysim
import pybullet as p
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from tqdm import tqdm

class Network(object):

	def __init__(self, Kp=-0.5, Ki=-0.5, nu=0.1):

		# Initialize state
		self.i = 0
		self.x = 0
		self.xdot = 0
		self.m = 0
		self.c = 0 # calibration coefficient

		# Set parameters
		self.Kp = Kp
		self.Ki = Ki 
		self.nu = nu # learning rate

		# Initialize storage
		self.x_series = []
		self.c_series = []
		self.m_series = []

	def step(self, pos, active=True):

		self.x = pos

		# Integrate error
		self.c += self.nu*self.m*self.Ki

		# Calculate PI control
		self.m = self.Kp*self.x + self.Ki*self.c

		self.i += 1
		self.x_series.append(self.x)
		self.c_series.append(self.c)
		self.m_series.append(self.m)

		return self.m

flyStartPos = [0,0,4]
flyStartOrn = p.getQuaternionFromEuler([0,0,0])
fly = flysim.Fly(flyStartPos, flyStartOrn, gui=0, apply_forces=1)
dt = fly.dt

# net = Network(Kp=-1e-2, Ki=-1e-4)
net = Network(Kp=-1e-2, Ki=0)

state = []
yaw_int = 0
yaw_int_series = []
for j in tqdm(range(2000)):
	fly.step_simulation()

	# Apply stabilizing external torque
	p.applyExternalTorque(
		fly.flyId,
		-1,
		# (0,8.5,0),
		(0,0,10),
		p.LINK_FRAME
		)

	# Extract state
	if j == 0:
		x_last, y_last, z_last = (0,0,0)
		roll_last, pitch_last, yaw_last = (0,0,0)
	else:
		x_last, y_last, z_last = (x,y,z)
		roll_last, pitch_last, yaw_last = (roll,pitch,yaw)

	pos = p.getBasePositionAndOrientation(fly.flyId)[0]
	x, y, z = pos
	z -= 4
	dx, dy, dz = (x-x_last)/dt, (y-y_last)/dt, (z-z_last)/dt

	orn = p.getBasePositionAndOrientation(fly.flyId)[1]
	roll, pitch, yaw = p.getEulerFromQuaternion(orn)
	droll, dpitch, dyaw = (roll-roll_last)/dt, (pitch-pitch_last)/dt, (yaw-yaw_last)/dt
	
	yaw_int += yaw*1e-4
	yaw_int_series.append(yaw_int)

	state.append([x,y,z,roll,pitch,yaw])

	# Apply orientation stabilization
	Kp = 1e6
	Kd = 1e3
	p.applyExternalTorque(
		fly.flyId,
		-1,
		(-Kp*roll-Kd*droll,-Kp*pitch-Kd*dpitch,0),
		p.WORLD_FRAME
		)

	# Apply position stabilization
	Kp = 1e6
	Kd = 1e4
	p.applyExternalForce(
		fly.flyId,
		-1,
		(-Kp*x-Kd*dx,-Kp*y-Kd*dy,-Kp*z-Kd*dz),
		(0,0,0),
		p.WORLD_FRAME
		)

	# cmd = net.step(yaw)
	# cmd = np.clip(cmd, -5e-5, 5e-5)
	cmd = -yaw*1e-3 - yaw_int*1e-8
	fly.cmd = [0,0,0,0,0,cmd]

state = np.array(state)
plt.plot(state[:,5])
plt.plot(yaw_int_series)
# plt.plot(net.x_series)
# plt.plot(net.c_series)
# plt.plot(net.m_series)
# plt.legend(["x","c","m"])
plt.show()