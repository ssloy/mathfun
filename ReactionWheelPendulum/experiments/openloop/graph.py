#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import sys

[time, refcurrent, current, Qr, Q] = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1, unpack=True)

k  = 0.0369
Jr = 0.001388
Cr = 0.00001
I  = .25

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum run")
ax1.set_xlabel('Time, sec')

#dt = [time[i+1]-time[i] for i in range(len(time)-1)]
#ax1.plot(time[1:], dt, color='black',  label='dt')

#ax1.plot(time, refcurrent, color='green',   label='reference current, A')
#ax1.plot(time, current,    color='red',    label='measured current, A')

sQr = [ t*k*I/Cr + k*I*Jr/Cr**2*np.exp(-t*Cr/Jr) - k*I*Jr/Cr**2 for t in time]

ax1.plot(time,  Qr, color='blue',  label='RW angle, rad')
ax1.plot(time, sQr, color='red',   label='synth RW angle, rad')

ax1.legend()
plt.show()

