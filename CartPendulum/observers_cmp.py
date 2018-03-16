#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import sys

[time, U, targetX, X, Q] = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1, unpack=True)

def blur(x):
    tmp = x[:]
    for i in range(1,len(x)-1):
        x[i] = (tmp[i-1]+2*tmp[i]+tmp[i+1])/4


def finite_difference(time, x, nblur):
    x_dot = []
    for i in range(nblur):
        blur(x)
    for i in range(len(x)-1):
        x_dot.append((x[i+1]-x[i])/(time[i+1]-time[i]))
    return x_dot

fdX = finite_difference(time, X[:], 10)
fdQ = finite_difference(time, Q[:], 10)

'''
Ki = 9.268
fricpos =  3.986*.8
fricneg = -2.875*.8
'''

hatQ = Q[0]
hatX = X[0]
hatLPQ = 0.
hatLPX = 0.
PEBO_hatDQ = []
PEBO_hatDX = []


gainX = -20.
gainQ = -15.
ZX = X[0]*gainX
ZQ = Q[0]*gainQ
KKL_hatDQ = []
KKL_hatDX = []

for i in range(len(time)-1):
    dt = time[i+1]-time[i]
    mesQ = Q[i+1]
    mesX = X[i+1]
    cosQ = np.cos(mesQ)
    sinQ = np.sin(mesQ)
    u = U[i+1]

    '''   
    u = current[i+1]*Ki
    if X[i]!=X[i+1]:
        if X[i+1]>X[i]:
            u = u-fricpos
        else:
            u = u-fricneg
    else:
        if u>0:
            if u>fricpos:
                u = u-fricpos
            else:
                u = 0.
        else:
            if u<fricneg:
                u = u-fricneg
            else:
                u = 0.
    '''

    # PEBO observer
    A = 1./np.sqrt(731.775 - 152.361*cosQ*cosQ)
    hatDQ = 123.018*hatLPQ* A
    hatDX = 0.933*hatLPX - (11.512*hatLPQ*cosQ) * A
    diffX = hatDX + 50*(mesX - hatX)
    diffQ = 80*(mesQ - hatQ) + hatDQ
    diffLPX = 373.066*mesX - 373.066*hatX + .933*u + (4604.916*cosQ*(hatQ - mesQ)) * A;
    diffLPQ = (18452.7315*(mesQ - hatQ) + 129.832*sinQ - 11.512*u*cosQ) * A;
    hatX   = hatX   + dt*diffX
    hatQ   = hatQ   + dt*diffQ
    hatLPX = hatLPX + dt*diffLPX
    hatLPQ = hatLPQ + dt*diffLPQ
    PEBO_hatDQ.append(hatDQ)
    PEBO_hatDX.append(hatDX)

    # KKL observer
    A = 1./np.sqrt(7.3178 - 1.5236*cosQ*cosQ)
    eX = ZX - gainX*mesX
    eQ = ZQ - gainQ*mesQ
    hatDQ = 12.3018*eQ*A
    hatDX = 0.9327*eX - 1.1512*eQ*cosQ*A
    diffZQ = (12.9832*sinQ - 1.1512*u*cosQ)*A + gainQ*hatDQ
    diffZX = 0.9327*u + gainX*hatDX
    ZX = ZX + dt*diffZX
    ZQ = ZQ + dt*diffZQ
    KKL_hatDQ.append(hatDQ)
    KKL_hatDX.append(hatDX)


fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum run")
ax1.set_xlabel('Time, sec')

dt = [time[i+1]-time[i] for i in range(len(time)-1)]
ax1.plot(time[1:], dt, color='black',  label='dt')

#ax1.plot(time[1:], fdX, color='black',  label='finite difference speed, m/s')
#ax1.plot(time[1:], KKL_hatDX, color='red',     label='KKL, cart speed, m/s')
#ax1.plot(time[1:], PEBO_hatDX, color='green',  label='GESO, cart speed, m/s')

#ax1.plot(time[1:], fdQ, color='black',  label='finite difference speed, rad/s')
#ax1.plot(time[1:], KKL_hatDQ, color='magenta', label='KKL, pend speed, rad/s')
#ax1.plot(time[1:], PEBO_hatDQ, color='blue',   label='GESO, pend speed, rad/s')

ax1.legend()
plt.show()

