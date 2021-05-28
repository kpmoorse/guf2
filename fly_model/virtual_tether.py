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
		self.c = 0 # calibration coefficient

		# Set parameters
		self.Kp = Kp
		self.Ki = Ki 
		self.nu = nu # learning rate

	def step(self, pos, active=True):

		self.x = pos

		m = self.Kp*self.x + self.Ki*self.c

		self.c += self.nu*m*self.Ki
		self.i += 1
		return m

flyStartPos = [0,0,4]
flyStartOrn = p.getQuaternionFromEuler([0,0,0])
fly = flysim.Fly(flyStartPos, flyStartOrn, gui=1, apply_forces=1)
dt = fly.dt

net = Network(Ki=0.0)

state = []
for j in tqdm(range(1000)):
	fly.step_simulation()

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

	cmd = net.step(yaw/np.pi)*1e-3
	fly.cmd = [0,0,0,0,0,cmd]

state = np.array(state)
plt.plot(state[:,5])
plt.show()