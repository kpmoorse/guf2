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

# ustb = npread('data/pitch_unstable.csv')
# stb = npread('data/pitch_stable.csv')
# wb = np.arange(stb.shape[0])/100
# plt.plot(wb[:1000], ustb[:1000], linewidth=3)
# plt.plot(wb, stb, linewidth=3)
# plt.xlabel("Time (wingbeats)")
# plt.ylabel("Pitch angle (rad)")
# plt.legend(["Feedforward", "Feedback"])

plt.rcParams.update({"font.size":14})
traces = []
for i in range(5):
	traces.append(npread('data/pitch_{}.csv'.format(i)))
wb = np.arange(traces[0].shape[0])/100
for i in range(5):
	plt.plot(wb, traces[i]*180/np.pi, linewidth=3)
# plt.xlabel("Time (wingbeats)")
# plt.ylabel("Pitch angle (rad)")
plt.legend(["-90$^\circ$","-45$^\circ$","0$^\circ$","45$^\circ$","90$^\circ$"])

ax = plt.gca()
ax.spines["right"].set_visible(False)
ax.spines["top"].set_visible(False)
for spine in (ax.spines["left"], ax.spines["bottom"]):
	spine.set_position(["outward",5])
ax.spines["left"].set_bounds(plt.yticks()[0][1], plt.yticks()[0][-2])
ax.spines["bottom"].set_bounds(plt.xticks()[0][1], plt.xticks()[0][-2])

plt.show()

# ustb = npread('data/roll_unstable.csv')
# stb = npread('data/roll_stable.csv')
# wb = np.arange(stb.shape[0])/100
# plt.plot(wb[:1000], ustb[:1000], linewidth=3)
# plt.plot(wb, stb, linewidth=3)
# plt.xlabel("Time (wingbeats)")
# plt.ylabel("Roll angle (rad)")
# plt.legend(["Feedforward", "Feedback"])

# traces = []
# for i in range(4):
# 	traces.append(npread('data/roll_{}.csv'.format(i)))
# wb = np.arange(traces[0].shape[0])/100
# for i in range(4):
# 	if i<3:
# 		plt.plot(wb, traces[i], linewidth=3)
# 	else:
# 		plt.plot(wb[:1000], traces[i][:1000], linewidth=3)
# plt.xlabel("Time (wingbeats)")
# plt.ylabel("Roll angle (rad)")
# plt.legend(["2","1.75","1.5","1.25"])



# plt.show()