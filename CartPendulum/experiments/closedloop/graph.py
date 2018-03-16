#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import sys

[time, current, target, x, theta] = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1, unpack=True)
[time2, current2, target2, x2, theta2] = np.loadtxt(sys.argv[2], delimiter=',', skiprows=1, unpack=True)

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum run")
ax1.set_xlabel('Time, sec')

#ax1.plot(time, refcurrent, color='green',   label='reference current, A')
#ax1.plot(time, current,    color='red',    label='measured current, A')
ax1.plot(time, target,          color='blue',    label='target cart position, m')
ax1.plot(time, x,          color='green',    label='GESO, cart position, m')
ax1.plot(time2, x2,          color='red',    label='KKL, cart position, m')
#ax1.plot(time, theta,      color='magenta', label='pendulum angle, rad')

ax1.legend()
plt.show()

