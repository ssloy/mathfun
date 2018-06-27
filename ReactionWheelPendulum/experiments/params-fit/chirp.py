#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import sys
import math

[time, ref, I, Qr, Q] = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1, unpack=True)

g = 9.81
k = 0.0369
Jr = 0.001248
Jp = 0.003
mp = .7
lp = .15
mr = .35
lr = .22
J = Jp + mr*lr*lr + mp*lp*lp
ml = mp*lp + mr*lr

sQ  = [Q[0]]
dsQ = 0.

sQr = [Qr[0]]
dsQr = 0.

for i in range(1, len(time)):
    ddsQ  = ml*g/J*np.sin(sQ[-1]) - k/J*I[i]
    ddsQr = (J+Jr)/(J*Jr)*k*I[i] - ml*g/J*np.sin(sQ[-1])
    dt = time[i]-time[i-1]
    dsQ += ddsQ*dt
    dsQr += ddsQr*dt
    sQ.append(sQ[-1]+dsQ*dt)
    sQr.append(sQr[-1]+dsQr*dt)

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum run")
ax1.set_xlabel('Time, sec')

#dt = [time[i+1]-time[i] for i in range(len(time)-1)]
#ax1.plot(time[1:], dt, color='black',  label='dt')

ax1.plot(time,  ref)
ax1.plot(time,  I)
#ax1.plot(time,  Q,    label='measured angle, rad')
#ax1.plot(time,  sQ,   label='synth angle, rad')
#ax1.plot(time,  Qr,    label='measured rw angle, rad')
#ax1.plot(time,  sQr,    label='synth rw angle, rad')


ax1.legend()
plt.show()

