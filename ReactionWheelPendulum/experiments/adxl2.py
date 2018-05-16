#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import sys
import math

files = ["chirp-pink-rubber-foam.csv", "chirp-sponge.csv", "chirp-antistatic-sponge.csv"]
#files = ["chirp-hardmount.csv", "chirp-neoprene.csv", "chirp-sponge.csv", "chirp-pink-rubber-foam.csv", "chirp-antistatic-sponge.csv"]

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum run")
ax1.set_xlabel('Time, sec')
for file in files:
    [time, ref, amps, Qr, Q, ax, ay, bx, by] = np.loadtxt(file, delimiter=',', skiprows=1, unpack=True)

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

    a = 0
    b = 0
    z = 0
    for i in range(10):
        a = a + aQ[i]
        b = b + bQ[i]
        z = z + zQ[i]

    a = a/10
    b = b/10
    z = z/10

    for i in range(len(zQ)):
        aQ[i] = aQ[i] - a - np.pi
        bQ[i] = bQ[i] - b - np.pi
        zQ[i] = zQ[i] - z - np.pi


#    ax1.plot(time, Q,  label=file)
    ax1.plot(time, aQ, label=file)

ax1.legend()
plt.show()

