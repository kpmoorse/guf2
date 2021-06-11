import control
import numpy as np
import matplotlib.pyplot as plt

g = 9.8

# Define system matrices (inverted pendulum)
A = np.array([[0,1], [g,0]])
B = np.array([[0],[1]])
C = np.array([[1,0]])
D = np.array([[0]])

sys = control.ss(A,B,C,D)

# Synthesize lqr controller
ctl = control.lqr(sys,[[1,0], [0,1]], [[1]])
K = np.array(ctl[0])
print(K)
sys_ctl = control.ss(A-np.dot(B,K),B,C,D)

# Define simulation parameters
t = np.arange(0,1,0.01)
u = 10*np.sin(t*3*2*np.pi)
x0 = [[0],[0]]

# Run open-loop system from initial state
r = control.forced_response(sys, T=t, U=u, X0=x0, return_x=1)
t = r[0]
x1 = r[2][0]

# Run closed-loop system from initial state
rc = control.forced_response(sys_ctl, T=t, U=u, X0=x0, return_x=1)
tc = rc[0]
x1c = rc[2][0]

plt.plot(t,u/100)
plt.plot(t,x1)
plt.plot(tc,x1c)
# plt.ylim([-0.1,0.5])
plt.show()