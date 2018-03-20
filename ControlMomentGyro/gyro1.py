#!/usr/bin/python3

import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

Ib = 0.0190 # kg m^2
Jb = 0.0178
Ka = 0.0670
Ic = 0.0092
Jc = 0.0230
Kb = 0.0297
Id = 0.0148
Jd = 0.0273
Kc = 0.0188
J1 = Jc+Jd-Kc-Id
J2 = Ka+Jc+Jd+Kb
wd = 1.59 # 600 rpm

def derivative(state, amps):
    [qa,wa,qc,wc] = state
    tauc = amps
    return np.array([wa, (-J1*np.sin(2*qc)*wc*wa - Jd*np.cos(qc)*wc*wd)/(J2-J1*(np.cos(qc)**2)), wc, (tauc + Jd*np.cos(qc)*wa*wd + 0.5*J1*np.sin(2*qc)*(wa**2))/(Id+Ic)])

def rk4_step(yi, i, time, current): 
    h = time[i+1]-time[i]
    k1 = derivative(yi, current[i])*h
    k2 = derivative(yi+k1/2., (current[i]+current[i+1])/2.)*h
    k3 = derivative(yi+k2/2., (current[i]+current[i+1])/2.)*h
    k4 = derivative(yi+k3, current[i+1])*h
    return yi+k1/6.+k2/3.+k3/3.+k4/6.

def predict(time, current):
    sol = []
    yi = [0, 0, 1.57, 0] # qa wa qc wc
    for i in range(len(time)-1):
        yi = rk4_step(yi, i, time, current)
        sol.append(yi)
    sol.append(yi)
    return np.transpose(sol)

time = np.linspace(0,10,3000)
current = [-0.01 for t in time]

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum dynamics compared")
ax1.set_xlabel('Time, sec')

synthetic = predict(time, current)
#ax1.plot(time, current,  label='current, amp')
ax1.plot(time, synthetic[0,:], label='qa, rad')
ax1.plot(time, synthetic[2,:], label='qc, rad')

ax1.legend()
plt.show()

