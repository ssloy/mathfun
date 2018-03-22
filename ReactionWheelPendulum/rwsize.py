#!/usr/bin/python3

import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

g = 9.81
k = 0.0369
R = 0.608
U = 24.

lp = .35
mp = 0.350
Jp = 0.005

lr = .5
r = 0.057 # disk radius
v = 0.020*np.pi*r*r # volume
mr = v*2698.9
Jr = 0.0000181 + mr*r*r/2

J = Jp + mp*lp*lp + mr*lr*lr
ml = mp*lp + mr*lr

gr = []
gx = []
bestx = -1
bestr = -1

for r in np.arange(0., .2, 0.001):
    v = 0.020*np.pi*r*r # volume
    mr = v*2698.9
    Jr = 0.0000181 + mr*r*r/2

    J = Jp + mp*lp*lp + mr*lr*lr
    ml = mp*lp + mr*lr

    A = np.matrix([[0.,1.,0.],[ml*g/J, 0., k**2/(J*R)],[-ml*g/J, 0., -(J+Jr)/(J*Jr)*k**2/R]])
    B = np.matrix([[0.],[-k/(J*R)],[(J+Jr)/(J*Jr)*k/R]])
    [eigenvalues, eigenvectors] = np.linalg.eig(A)
    V = np.matrix(eigenvectors)
    ymax = (V.I*B).item(0,0)*U/eigenvalues[0]
    xmax = np.abs(ymax/V.I.item(0,0))
    if (bestx<xmax):
        bestr = r
        bestx = xmax
    gr.append(r*2.)
    gx.append(xmax)

print("Best diameter: ", 2.*bestr, " max angle: ", bestx)

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Max possible angle to achieve a stabilization")
ax1.set_ylabel('angle, rad')
ax1.set_xlabel('reaction wheel diameter, m (aluminium disk, 20mm thickness)') 
ax1.plot(gr,gx)
plt.show()



A = np.matrix([[0.,1.,0.],[ml*g/J, 0., k**2/(J*R)],[-ml*g/J, 0., -(J+Jr)/(J*Jr)*k**2/R]])
B = np.matrix([[0.],[-k/(J*R)],[(J+Jr)/(J*Jr)*k/R]])

def integrate(x0,time):
    synthetic = [x0.copy().transpose().tolist()[0]]
    xi = x0.copy()
    for i in range(len(time)-1):
        xi = xi + (A*xi+B*U)*(time[i+1]-time[i])
        synthetic.append(xi.copy().transpose().tolist()[0])
    return np.transpose(synthetic)

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum dynamics")
ax1.set_xlabel('Time, sec')

x0 = np.matrix([0.24, 0., 0.]).transpose() # theta thetadot
time = np.linspace(0, 2, 2000)
simu = integrate(x0, time)
ax1.plot(time, simu[0,:], label='angle, rad')
ax1.legend()
plt.show()


