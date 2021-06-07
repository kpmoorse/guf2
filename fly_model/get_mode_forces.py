import flysim
import pybullet as p
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from tqdm import tqdm

def mix(color1, color2, val):
	res = []
	for c1, c2 in zip(color1,color2):
		res.append(c1*(1-val)+c2*(val))
	return res

def npwrite(arr, file):
    df = pd.DataFrame(arr)
    df.to_csv(file, header=False, index=False)

flyStartPos = [0,0,4]
flyStartOrn = p.getQuaternionFromEuler([0,0,0])
fly = flysim.Fly(flyStartPos, flyStartOrn, gui=False, apply_forces=False)

cmd_mask = np.eye(6)

# Pass over startup transients
for j in range(100):
    fly.step_simulation()
fly.reset()

N = 5
mag_list = np.arange(-N,N+1)*1e-5
# mag_list = mag_list[::-1]
forces = []
torques = []
for i in range(6):
    forces.append(np.empty((3,0)))
    torques.append(np.empty((3,0)))

# Loop over orthogonal commands
for i, cmd in enumerate(tqdm(cmd_mask)):
    # Loop over command magnitudes
    for mag in tqdm(mag_list, leave=False):
        fly.cmd = cmd*mag
        # Loop over simulation timesteps
        for j in tqdm(range(200), leave=False):
            fly.step_simulation()
        # plt.plot(fly.forces)
        # plt.plot(fly.torques)
        # plt.show()
        forces[i] = np.hstack((forces[i], np.mean(fly.forces[100:], axis=0)[:,None]))
        torques[i] = np.hstack((torques[i], np.mean(fly.torques[100:], axis=0)[:,None]))
        fly.reset()
# forces = np.transpose(np.array(forces))
# print(forces)

max_val = 0
for f, t in zip(forces, torques):
    max_val = max(max_val, np.max(np.abs(f)), np.max(np.abs(t)))
max_val *= 1.1

# # Plot Fy mode
# i = 1
# for j, f in enumerate(forces[i]):
#     line, = plt.plot(mag_list*1e5, f-f[N], '.-')
#     if i == j:
#         plt.setp(line, linewidth=3)
#     else:
#         plt.setp(line, linestyle='--')
# for j, f in enumerate(torques[i]):
#     line, = plt.plot(mag_list*1e5, f-f[N], '.-')
#     plt.setp(line, linestyle='--')
# plt.xlabel("Mode strength")
# plt.ylabel("Response strength")
# plt.legend(["$F_x$","$F_y$","$F_z$","$M_x$","$M_y$","$M_z$"])
# ax = plt.gca()
# ax.spines["right"].set_visible(False)
# ax.spines["top"].set_visible(False)
# for spine in (ax.spines["left"], ax.spines["bottom"]):
# 	spine.set_position(["outward",5])
# ax.spines["left"].set_bounds(plt.yticks()[0][1], plt.yticks()[0][-2])
# ax.spines["bottom"].set_bounds(plt.xticks()[0][1], plt.xticks()[0][-2])
# plt.tight_layout()
# plt.show()

# # Plot Mx mode
# i = 0
# for j, f in enumerate(forces[i+3]):
#     line, = plt.plot(mag_list*1e5, f-f[N], '.-')
#     plt.setp(line, linestyle='--')
# for j, f in enumerate(torques[i+3]):
#     line, = plt.plot(mag_list*1e5, f-f[N], '.-')
#     if i == j:
#         plt.setp(line, linewidth=3)
#     else:
#         plt.setp(line, linestyle='--')
# plt.xlabel("Mode strength")
# plt.ylabel("Response strength")
# plt.legend(["$F_x$","$F_y$","$F_z$","$M_x$","$M_y$","$M_z$"])
# ax = plt.gca()
# ax.spines["right"].set_visible(False)
# ax.spines["top"].set_visible(False)
# for spine in (ax.spines["left"], ax.spines["bottom"]):
# 	spine.set_position(["outward",5])
# ax.spines["left"].set_bounds(plt.yticks()[0][1], plt.yticks()[0][-2])
# ax.spines["bottom"].set_bounds(plt.xticks()[0][1], plt.xticks()[0][-2])
# plt.tight_layout()
# plt.show()

# # Plot My mode
# i = 1
# for j, f in enumerate(forces[i+3]):
#     line, = plt.plot(mag_list*1e5, f-f[N], '.-')
#     plt.setp(line, linestyle='--')
# for j, f in enumerate(torques[i+3]):
#     line, = plt.plot(mag_list*1e5, f-f[N], '.-')
#     if i == j:
#         plt.setp(line, linewidth=3)
#     else:
#         plt.setp(line, linestyle='--')
# plt.xlabel("Mode strength")
# plt.ylabel("Response strength")
# plt.legend(["$F_x$","$F_y$","$F_z$","$M_x$","$M_y$","$M_z$"])
# ax = plt.gca()
# ax.spines["right"].set_visible(False)
# ax.spines["top"].set_visible(False)
# for spine in (ax.spines["left"], ax.spines["bottom"]):
# 	spine.set_position(["outward",5])
# ax.spines["left"].set_bounds(plt.yticks()[0][1], plt.yticks()[0][-2])
# ax.spines["bottom"].set_bounds(plt.xticks()[0][1], plt.xticks()[0][-2])
# plt.tight_layout()
# plt.show()

# Plot forces
for i in range(3):
    ax = plt.subplot(2,3,i+1)
    for j, f in enumerate(forces[i]):
        line, = plt.plot(mag_list*1e5, f-f[N], '.-')
        if i == j:
            plt.setp(line, linewidth=3)
        else:
            plt.setp(line, linestyle='--')
    for j, f in enumerate(torques[i]):
        line, = plt.plot(mag_list*1e5, f-f[N], '.-')
        plt.setp(line, linestyle='--')

    # plt.ylim([-max_val, max_val])
    plt.yticks(rotation=45)
    if i>0:
        # plt.yticks([])
        pass
    else:
        plt.ylabel("Response strength")
    plt.title(["$F_x$","$F_y$","$F_z$"][i])

    ax = plt.gca()
    ax.spines["right"].set_visible(False)
    ax.spines["top"].set_visible(False)
    for spine in (ax.spines["left"], ax.spines["bottom"]):
        spine.set_position(["outward",5])
    ax.spines["left"].set_bounds(plt.yticks()[0][1], plt.yticks()[0][-2])
    ax.spines["bottom"].set_bounds(plt.xticks()[0][1], plt.xticks()[0][-2])

# Plot torques
for i in range(3):
    plt.subplot(2,3,i+4)
    for j, f in enumerate(forces[i+3]):
        line, = plt.plot(mag_list*1e5, f-f[N], '.-')
        plt.setp(line, linestyle='--')
    for j, f in enumerate(torques[i+3]):
        line, = plt.plot(mag_list*1e5, f-f[N], '.-')
        if i == j:
            plt.setp(line, linewidth=3)
        else:
            plt.setp(line, linestyle='--')

    # plt.ylim([-max_val, max_val])
    plt.xlabel("Mode strength")
    plt.yticks(rotation=45)
    if i>0:
        # plt.yticks([])
        pass
    else:
        plt.ylabel("Response strength")
    plt.title(["$M_x$","$M_y$","$M_z$"][i])

    ax = plt.gca()
    ax.spines["right"].set_visible(False)
    ax.spines["top"].set_visible(False)
    for spine in (ax.spines["left"], ax.spines["bottom"]):
        spine.set_position(["outward",5])
    ax.spines["left"].set_bounds(plt.yticks()[0][1], plt.yticks()[0][-2])
    ax.spines["bottom"].set_bounds(plt.xticks()[0][1], plt.xticks()[0][-2])

plt.tight_layout()
plt.show()
# npwrite(f,'temp/fz_p1.csv')
# plt.plot(f)
# plt.show()