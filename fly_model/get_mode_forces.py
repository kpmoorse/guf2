import flysim
import pybullet as p
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def npwrite(arr, file):
    df = pd.DataFrame(arr)
    df.to_csv(file, header=False, index=False)

magnitude = np.arange(-2e-5,3e-5,1e-5)
print(magnitude)
response = []
flies = []

mag = 1e-5

flyStartPos = [0,0,4]
flyStartOrn = p.getQuaternionFromEuler([0,0,0])
fly = flysim.Fly(flyStartPos, flyStartOrn, gui=False, apply_forces=False, cmd=(0,0,mag,0,0,0))

for j in range(200):
	fly.step_simulation()

f = fly.forces[:,2]
npwrite(f,'temp/fz_p1.csv')
# plt.plot(f)
# plt.show()