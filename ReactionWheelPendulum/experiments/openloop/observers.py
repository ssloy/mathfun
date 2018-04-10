#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import sys
import math

[time, ref, I, Qr, Q] = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1, unpack=True)

a = .75
k1 = 7.
k2 = 12.

g = 9.81
k = 0.0369
Jr = 0.001248
J = .7*.160**2
ml = .7*.160

hatXp1 = Q[0]
hatXp2 = 0.

hatXr1 = Qr[0]
hatXr2 = 0.

hatQ  = []
hatQr = []

for i in range(1, len(time)):
    dt = time[i]-time[i-1]

    ep = hatXp1 - Q[i]
    signp = 1.
    if (ep<0):
        signp = -1.
    dhatXp1 = hatXp2                          - k1*signp*math.pow(math.fabs(ep),     a) 
    dhatXp2 = -k/J*I[i] + ml*g/J*np.sin(Q[i]) - k2*signp*math.pow(math.fabs(ep), 2*a-1)
    hatXp1 = hatXp1 + dt*dhatXp1
    hatXp2 = hatXp2 + dt*dhatXp2
    hatQ.append(hatXp1)

    er = hatXr1 - Qr[i]
    signr = 1.
    if (er<0):
        signr = -1.
    dhatXr1 = hatXr2                                     - k1*signr*math.pow(math.fabs(er),     a) 
    dhatXr2 = (J+Jr)/(J*Jr)*k*I[i] - ml*g/J*np.sin(Q[i]) - k2*signr*math.pow(math.fabs(er), 2*a-1)
    hatXr1 = hatXr1 + dt*dhatXr1
    hatXr2 = hatXr2 + dt*dhatXr2
    hatQr.append(hatXr1)

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum run")
ax1.set_xlabel('Time, sec')

dt = [time[i+1]-time[i] for i in range(len(time)-1)]
#ax1.plot(time[1:], dt, color='black',  label='dt')

ax1.plot(time,        Qr,  color='green', label='RW angle, rad')
ax1.plot(time[1:],  hatQr, color='black', label='observer RW angle, rad')
ax1.plot(time,         Q,  color='red',   label='pend angle, rad')
ax1.plot(time[1:],  hatQ,  color='blue',  label='observer pend angle, rad')

ax1.legend()
plt.show()

