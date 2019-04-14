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

dt = .001
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
kc = 1.62 # Nm/A

A = np.matrix([[0., 1., 0., 0.], [m*g*l/(l**2*m+Kc+Jb+Id), 0., 0., Jd*wd/(l**2*m+Kc+Jb+Id)], [0., 0., 0., 1.], [0., -Jd*wd/(Ic+Id), 0., 0.]])
B = np.matrix([[0.],[0.],[0.],[1./(Ic+Id)]])
Q = np.matrix("100. 0. 0. 0.; 0. 10. 0. 0. ; 0. 0. 10. 0.; 0. 0. 0. 1. ")
R = np.matrix("100.")

print("A:", A)
print("B:", B)

K = lqr(A,B,Q,R)
print("double K[] = {%f, %f, %f, %f};" % (K[0,0], K[0,1], K[0,2], K[0,3]))

nsteps = 3000
time = np.linspace(0, nsteps*dt, nsteps, endpoint=True)

QB = []
QC = []
I  = []

qb = -np.pi/4. + .3
qc = np.pi/2.
wb = 0.
wc = 0.

for t in time:
    Tau = -K*np.matrix([[qb+np.pi/4],[wb],[qc-np.pi/2],[wc]])
    if (Tau[0,0]>5.*kc):
        Tau[0,0] = 5.*kc

    if (Tau[0,0]<-5.*kc):
        Tau[0,0] = -5.*kc

    M = np.matrix([[(Kc-Jd-Jc+Id)*np.sin(qc)**2+l**2*m+Jd+Jc+Jb,0],[0,Id+Ic]])
    C = np.matrix([[((2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qc)*np.sin(qc)*wc)/2,((2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qc)*np.sin(qc)*wb-Jd*np.sin(qc)*wd)/2],[(Jd*np.sin(qc)*wd+((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qc)*np.sin(qc)*wb)/2,0]])
    G = np.matrix([[-(Jd*np.sin(qc)*wc*wd+2*g*l*m*np.sin((4*qb+np.pi)/4))/2],[(Jd*np.sin(qc)*wb*wd)/2]])

    ddQ = M.I*(np.matrix([[0.],[Tau[0,0]]]) - C*np.matrix([[wb], [wc]]) - G)

    wb = wb + dt*ddQ[0,0]
    wc = wc + dt*ddQ[1,0]

    qb = qb + dt*wb
    qc = qc + dt*wc

    QB.append(qb)
    QC.append(qc)
    I.append(Tau[0,0]/kc)

plt.plot(time, QB, label='qb, rad')
plt.plot(time, QC, label='qc, rad')
plt.plot(time, I, label='control current, A')

plt.legend(loc='upper right')
plt.grid()
plt.show()

