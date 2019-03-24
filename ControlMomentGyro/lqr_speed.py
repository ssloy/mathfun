#!/usr/bin/python3

import numpy as np
import scipy.linalg
from scipy import signal
import matplotlib.pyplot as plt

def lqr(A,B,Q,R):
    """Solve the continuous time lqr controller.
    dx/dt = A x + B u
    cost = integral x.T*Q*x + u.T*R*u
    """
    #ref Bertsekas, p.151
    #first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
    #compute the LQR gain
    K = np.matrix(scipy.linalg.inv(R)*(B.T*X))
    eigVals, eigVecs = scipy.linalg.eig(A-B*K)
    return K

dt = .0001
g = 9.81

m = 2.62

Ic = 0.001
Jc = 0.000260
Kc = 0.000987

Id = 0.000564
Jd = 0.0011
Kd = 0.000564

Ib = 0.01045
Jb = 0.01312
Kb = 0.01358

wd = 223.6

l = .1344
#k = 1.62 # Nm/A

J1 = Jd + Jb + Kc + m*l*l
J2 = Jc - Id + Jd - Kc
m0 = m*g*l/np.sqrt(2)

A = np.matrix([[0., 1., 0.], [m*g*l/J1, 0., 0.], [0., 0., 0.]])
B = np.matrix([[0.],[Jd*wd/J1],[1.]])
Q = np.matrix("100. 0. 0.; 0. 10. 0. ; 0. 0. 100.")
R = np.matrix("100.")


print("A:", A)
print("B:", B)

K = lqr(A,B,Q,R)
print("double K[] = {%f, %f, %f};" % (K[0,0], K[0,1], K[0,2]))

nsteps = 30000
time = np.linspace(0, nsteps*dt, nsteps, endpoint=True)

QB  = []
QC  = []
U  = []

qb = -np.pi/4. + .2
qc = np.pi/2.
wb = 0.
wc = 0.

for t in time:
    wc = 0
    if (t>0):
        wc = -(K[0,0]*(qb+np.pi/4) + K[0,1]*wb + K[0,2]*(qc-np.pi/2))

    dwb = (J2*np.sin(qc*2)*wc*wb + Jd*np.sin(qc)*wc*wd + m0*(np.sin(qb)+np.cos(qb)))/(J1+J2*(np.cos(qc)**2))
    wb = wb + dt*dwb
    qb = qb + dt*wb
    qc = qc + dt*wc

    QB.append(qb)
    QC.append(qc)
    U.append(wc)

plt.plot(time, QB, label='pendulum angle, radians')
plt.plot(time, QC, label='gimbal angle, radians')
plt.plot(time, U, label='gimbal speed, rad/s')

plt.legend(loc='upper right')
plt.grid()
plt.show()

