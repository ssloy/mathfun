#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import sys

[time, rps, a,b,c, ref] = np.loadtxt(sys.argv[1], delimiter=',', unpack=True)
time = time/1000.

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("bldc esc run")
ax1.set_xlabel('Time, sec')

ref_si = [(r-100.)/100.*400.*10. / 60.*6.28 for r in ref]

ax1.plot(time,  rps,    color='blue',  label='real speed, rad/s')
ax1.plot(time,  ref_si, color='red',   label='reference speed, rad/s')

ax1.legend()
plt.show()

