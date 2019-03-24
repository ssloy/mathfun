#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import sys
import math

[time, refI, I, qb, wb, qc, wc] = np.loadtxt(sys.argv[1], delimiter=',', unpack=True)

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum run")
ax1.set_xlabel('Time, sec')

dt = [time[i+1]-time[i] for i in range(len(time)-1)]
#ax1.plot(time[1:], dt, color='black',  label='dt')

ax1.plot(time, qb, label='qb')
ax1.plot(time, wb, label='wb')
ax1.plot(time, qc, label='qc')
ax1.plot(time, wc, label='wc')

ax1.plot(time, refI, label='ref I')
#ax1.plot(time, I, label='I')


ax1.legend()
plt.show()

