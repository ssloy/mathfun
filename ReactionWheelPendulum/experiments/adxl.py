#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import sys
import math

[time, ref, amps, Qr, Q, ax, ay, bx, by] = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1, unpack=True)

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum run")
ax1.set_xlabel('Time, sec')

dt = [time[i+1]-time[i] for i in range(len(time)-1)]
#ax1.plot(time[1:], dt, color='black',  label='dt')
#Q = Q+3.1415

aQ = [math.atan2(-ax[i], ay[i]) for i in range(len(ax))]
bQ = [math.atan2(-bx[i], by[i]) for i in range(len(ax))]
mu = 2.0
zQ = [math.atan2((ax[i]-mu*bx[i]), -(ay[i]-mu*by[i])) for i in range(len(ax))]


for i in range(len(zQ)):
    while aQ[i]+np.pi>np.pi:
        aQ[i] = aQ[i] - 2*np.pi
    while zQ[i]+np.pi<-np.pi:
        aQ[i] = aQ[i] + 2*np.pi

    while bQ[i]+np.pi>np.pi:
        bQ[i] = bQ[i] - 2*np.pi
    while zQ[i]+np.pi<-np.pi:
        bQ[i] = bQ[i] + 2*np.pi

    while zQ[i]+np.pi>np.pi:
        zQ[i] = zQ[i] - 2*np.pi
    while zQ[i]+np.pi<-np.pi:
        zQ[i] = zQ[i] + 2*np.pi

ax1.plot(time, aQ,  color='blue',   label='raw adxl345 #1 readings')
ax1.plot(time, bQ,  color='green',  label='raw adxl345 #2 readings')
ax1.plot(time, zQ,  color='black',  label='adxl345 pendulum angle, rad')
ax1.plot(time,  Q,  color='red',    label='encoder pendulum angle, rad')
ax1.plot(time, amps,  color='magenta',    label='current, A')

ax1.legend()
plt.show()

