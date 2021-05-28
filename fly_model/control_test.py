import flysim
import pybullet as p
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from tqdm import tqdm

def npwrite(arr, file):
    df = pd.DataFrame(arr)
    df.to_csv(file, header=False, index=False)

flyStartPos = [0,0,4]
flyStartOrn = p.getQuaternionFromEuler([0,0,0])
fly = flysim.Fly(flyStartPos, flyStartOrn, gui=1, apply_forces=1)

a = 0.01
pitch_raw = []
pitch_dot = []
roll_raw = []
roll_dot = []
cmd_target = [0,0,0,0,0,0]
n = 1500
for j in tqdm(range(3000)):
    fly.step_simulation()

    # Extract orientation
    orn = p.getBasePositionAndOrientation(fly.flyId)[1]
    pitch_raw.append(p.getEulerFromQuaternion(orn)[1])
    roll_raw.append(p.getEulerFromQuaternion(orn)[0])

    if j == 0:
        pitch_dot.append(0)
        roll_dot.append(0)
    else:
        pitch_dot.append((pitch_raw[-1] - pitch_raw[-2])*fly.dt)
        roll_dot.append((roll_raw[-1] - roll_raw[-2])*fly.dt)

    # Apply proportional control
    if fly.i % 100 == 0:
        cmd_target = [
            0,
            0,
            0,
            0,
            # np.clip(-roll_raw[-1]*20e-5, -5e-5, 5e-5), # Roll control
            # np.clip(-roll_raw[-1]*25e-5-roll_dot[-1]*10e0, -5e-5, 5e-5), # Roll control
            # 0,
            # np.clip(-pitch_raw[-1]*5e-5, -8e-5, 8e-5), # Pitch control
            np.clip(-pitch_raw[-1]*5e-5-pitch_dot[-1]*5e0, -8e-5, 8e-5), # Pitch control
            0
        ]
    fly.cmd = [0.5*i + 0.5*j for (i,j) in zip(fly.cmd, cmd_target)]
    fly.cmd = cmd_target

# wbfVal=1
# clipVal=2
# npwrite(roll_raw, "data/roll_wbf{}_{:02d}.csv".format(wbfVal,clipVal))
# npwrite(pitch_raw, "data/pitch_wbf{}_{:02d}.csv".format(wbfVal,clipVal))
# plt.plot(pitch_raw)
# plt.plot(roll_raw)
# plt.show()