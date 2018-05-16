#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import sys
import math

[time, I, Qr, Q, Qa] = np.loadtxt(sys.argv[1], delimiter=',', unpack=True)

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum run")
ax1.set_xlabel('Time, sec')

dt = [time[i+1]-time[i] for i in range(len(time)-1)]
#ax1.plot(time[1:], dt, color='black',  label='dt')

ax1.plot(time,  Q,  color='red',   label='pend angle, rad')
ax1.plot(time, Qa,  color='blue',  label='aQ')

ax1.legend()
plt.show()

