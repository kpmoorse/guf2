import numpy as np
import matplotlib.pyplot as plt

def plt_std_form():
	ax = plt.gca()
	ax.spines["right"].set_visible(False)
	ax.spines["top"].set_visible(False)
	for spine in (ax.spines["left"], ax.spines["bottom"]):
		spine.set_position(["outward",5])
	ax.spines["left"].set_bounds(plt.yticks()[0][1], plt.yticks()[0][-2])
	ax.spines["bottom"].set_bounds(plt.xticks()[0][1], plt.xticks()[0][-2])

class Network(object):

	def __init__(self, a=-0.5, nu=0):

		# Initialize state
		self.i = 0
		self.h = 0
		self.hdot = 0

		# Set parameters
		self.a = a
		self.c = 0
		self.nu = nu # learning rate
		self.v0 = 0.05

		self.h_list = [self.h]

	def step(self):

		if self.i < 50:
			m = self.a*self.h + self.c
		else:
			m = self.c
		self.c = self.nu*m + (1-self.nu)*self.c
		self.hdot = m + self.v0
		self.h += self.hdot
		self.i += 1

		self.h_list.append(self.h)


if __name__ == '__main__':

	net = Network(0, 0)
	for i in range(100):
		net.step()
	plt.plot(net.h_list, linewidth=3)

	net = Network(-0.5, 0)
	for i in range(100):
		net.step()
	plt.plot(net.h_list, linewidth=3)

	net = Network(-0.5, 0.075)
	for i in range(100):
		net.step()
	plt.plot(net.h_list, linewidth=3)

	plt.axhline(0, color='k', ls='--')
	plt.axvline(50, color='k', ls='--')
	plt.ylim((-0.05,0.55))
	plt.xlabel("Time")
	plt.ylabel("Position")
	# plt.legend(["Uncontrolled"],loc="upper right")
	# plt.legend(["Uncontrolled", "Proportional"],loc="upper right")
	plt.legend(["Uncontrolled", "Proportional", "Calibrated"],loc="upper right")
	plt_std_form()
	plt.show()