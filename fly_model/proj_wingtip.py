import pybullet as p
import time
import pybullet_data
import pandas as pd
import numpy as np
from numpy import sin, cos, pi
import csv
from qfunc import *
import matplotlib.pyplot as plt
import modes

a, X, b = modes.read_modes()

def plot_kin(kin, c="k"):
    phi, theta, eta = kin.T
    plt.plot(phi, theta, c=c)
    plt.plot(phi[0], theta[0], '.', c=c)
    l = 0.04
    for i in range(20):
        x0 = phi[i*5]
        y0 = theta[i*5]
        e0 = eta[i*5]
        plt.plot(
            [x0+l*cos(e0),x0-l*cos(e0)],
            [y0-l*sin(e0),y0+l*sin(e0)],
            c=c
        )

    ax = plt.gca()
    ax.spines["right"].set_visible(False)
    ax.spines["top"].set_visible(False)
    ax.spines["left"].set_visible(False)
    ax.spines["bottom"].set_visible(False)
    plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    plt.tick_params(axis='y', which='both', right=False, left=False, labelleft=False)

    ax.set_aspect('equal')
    plt.tight_layout()

# Extract wing-wise kinematics
kin0 = modes.calc_kinematics(a, X, b, [0,0,0,0,0,0])
kin0L = kin0[:,:3]
kin0R = kin0[:,3:]

# # Roll
# kinY = modes.calc_kinematics(a, X, b, [0,0,0,1e-5,0,0])
# kinYL = kinY[:,:3]
# kinYR = kinY[:,3:]
# kinY2 = modes.calc_kinematics(a, X, b, [0,0,0,-1e-5,0,0])
# kinY2L = kinY2[:,:3]
# kinY2R = kinY2[:,3:]

# # Pitch
# kinY = modes.calc_kinematics(a, X, b, [0,0,0,0,1e-5,0])
# kinYL = kinY[:,:3]
# kinYR = kinY[:,3:]
# kinY2 = modes.calc_kinematics(a, X, b, [0,0,0,0,-1e-5,0])
# kinY2L = kinY2[:,:3]
# kinY2R = kinY2[:,3:]

# Yaw
kinY = modes.calc_kinematics(a, X, b, [0,0,0,0,0,3e-5])
kinYL = kinY[:,:3]
kinYR = kinY[:,3:]
kinY2 = modes.calc_kinematics(a, X, b, [0,0,0,0,0,-3e-5])
kinY2L = kinY2[:,:3]
kinY2R = kinY2[:,3:]

# # Z
# kinY = modes.calc_kinematics(a, X, b, [0,0,3e-5,0,0,0])
# kinYL = kinY[:,:3]
# kinYR = kinY[:,3:]
# kinY2 = modes.calc_kinematics(a, X, b, [0,0,-3e-5,0,0,0])
# kinY2L = kinY2[:,:3]
# kinY2R = kinY2[:,3:]

plt.subplot(121)
plot_kin(kin0L, "b")
plot_kin(kinYL, "r")
plot_kin(kinY2L, "g")
plt.gca().invert_xaxis()

plt.subplot(122)
plot_kin(kin0R, "b")
plot_kin(kinYR, "r")
plot_kin(kinY2R, "g")

plt.show()