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

#wd = 223.6

l = .1344
kd = 0.0369  # Nm/A

A = np.matrix([[0,1,0],[(g*l*m)/(l**2*m+Jc+Jb),0,0],[-(g*l*m)/(l**2*m+Jc+Jb),0,0]])
B = np.matrix([[0],[-1/(l**2*m+Jc+Jb)],[(l**2*m+Jd+Jc+Jb)/(Jd*l**2*m+(Jc+Jb)*Jd)]])

#Q = np.matrix("100. 0. 0. 0.; 0. 10. 0. 0. ; 0. 0. 10. 0.; 0. 0. 0. 1. ")
#R = np.matrix("100.")
Q = np.matrix("100. 0. 0.; 0. 10. 0. ; 0. 0. .1")
R = np.matrix("10000.")

print("A:", A)
print("B:", B)

K = lqr(A,B,Q,R)
print("double K[] = {%f, %f, %f};" % (K[0,0], K[0,1], K[0,2]))

nsteps = 3000
time = np.linspace(0, nsteps*dt, nsteps, endpoint=True)

QB = []
WD = []
I  = []

qb = -np.pi/4. + .05
wb = 0.
wd = 0.

for t in time:
    Tau = -K*np.matrix([[qb+np.pi/4],[wb],[wd]])
    if (Tau[0,0]>5.*kd):
        Tau[0,0] = 5.*kd

    if (Tau[0,0]<-5.*kd):
        Tau[0,0] = -5.*kd

    M = np.matrix([[l**2*m+Jd+Jc+Jb,Jd],[Jd,Jd]])
    C = np.matrix([[0,0],[0,0]])
    G = np.matrix([[-g*l*m*np.sin((4*qb+np.pi)/4)],[0]])

    ddQ = M.I*(np.matrix([[0.],[Tau[0,0]]]) - C*np.matrix([[wb], [wd]]) - G)

    wb = wb + dt*ddQ[0,0]
    wd = wd + dt*ddQ[1,0]
    qb = qb + dt*wb

    QB.append(100*qb)
    WD.append(wd)
    I.append(Tau[0,0]/kd)

plt.plot(time, QB, label='100*qb, rad')
plt.plot(time, WD, label='wd, rad/s')
plt.plot(time, I, label='control current, A')

plt.legend(loc='upper right')
plt.grid()
plt.show()
