import flysim
import pybullet as p
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from tqdm import tqdm

def npread(file, sep=',', header=None):
    df = pd.read_csv(file, sep=sep, header=header)
    arr = np.asarray(df)
    return arr

def plt_std_form():
	ax = plt.gca()
	ax.spines["right"].set_visible(False)
	ax.spines["top"].set_visible(False)
	for spine in (ax.spines["left"], ax.spines["bottom"]):
		spine.set_position(["outward",5])
	ax.spines["left"].set_bounds(plt.yticks()[0][1], plt.yticks()[0][-2])
	ax.spines["bottom"].set_bounds(plt.xticks()[0][1], plt.xticks()[0][-2])

wb = np.arange(3000)/100

for i in [4,6,8,10][::-1]:
	# Plot roll
	plt.subplot(221)
	plt.plot(wb,npread("data/roll_wbf{}_{:02d}.csv".format(0,i)))
	plt.subplot(223)
	plt.plot(wb,npread("data/pitch_wbf{}_{:02d}.csv".format(0,i)))

	# Plot pitch
	plt.subplot(222)
	plt.plot(wb,npread("data/roll_wbf{}_{:02d}.csv".format(1,i)))
	plt.subplot(224)
	plt.plot(wb,npread("data/pitch_wbf{}_{:02d}.csv".format(1,i)))

i = 2
plt.subplot(221)
plt.plot(wb[:1000],npread("data/roll_wbf{}_{:02d}.csv".format(0,i))[:1000])
plt.legend(["1.0","0.8","0.6","0.4","0.2"])
plt.title("Standard WBF")
plt.ylabel("Roll angle")
plt.ylim([-0.7,0.05])
plt.gca().spines["bottom"].set_visible(False)
plt_std_form()
plt.xticks([])

plt.subplot(223)
plt.plot(wb[:1000],npread("data/pitch_wbf{}_{:02d}.csv".format(0,i))[:1000])
plt.xlabel("Time (wingbeats)")
plt.ylabel("Pitch angle")
plt.ylim([-0.15,1.1])
plt_std_form()

plt.subplot(222)
plt.plot(wb,npread("data/roll_wbf{}_{:02d}.csv".format(1,i)))
plt.title("Increased WBF (+20%)")
plt.ylim([-0.7,0.05])
plt.gca().spines["left"].set_visible(False)
plt.gca().spines["bottom"].set_visible(False)
plt_std_form()
plt.yticks([])
plt.xticks([])

plt.subplot(224)
plt.plot(wb,npread("data/pitch_wbf{}_{:02d}.csv".format(1,i)))
plt.xlabel("Time (wingbeats)")
plt.ylim([-0.15,1.1])
plt.gca().spines["left"].set_visible(False)
plt_std_form()
plt.yticks([])

plt.tight_layout()
plt.show()