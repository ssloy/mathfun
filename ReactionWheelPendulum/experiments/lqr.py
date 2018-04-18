#!/usr/bin/python3

import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

def dlqr(A,B,Q,R):
    """
    Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    # first, solve the ricatti equation
    P = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
    # compute the LQR gain
    K = np.matrix(scipy.linalg.inv(B.T*P*B+R)*(B.T*P*A))
    return -K


dt = .005
g = 9.81
k = 0.0369
Jr = 0.001248
J = .7*.160**2
ml = .7*.160

A = np.matrix([[1.,dt,0.],[ml*g/J*dt, 1., 0.],[-ml*g/J*dt, 0., 1.]])
B = np.matrix([[0.],[-k/J*dt],[(J+Jr)*k/(J*Jr)*dt]])

print(A)
print(B)

Q = np.matrix("1 0 0; 0 .1 0 ; 0 0 .0001")
R = np.matrix(".001")

K = dlqr(A,B,Q,R)
print("double K[] = {%f, %f, %f};" % (K[0,0], K[0,1], K[0,2]))

nsteps = 2000
time = np.linspace(0, 10, nsteps, endpoint=True)
xk = np.matrix(".15 ; 0 ; 0")

Q  = []
I  = []

for t in time:
    ik = K*xk
    if (ik[0,0]>5.):
        ik[0,0] = 5.

    if (ik[0,0]<-5.):
        ik[0,0] = -5.

    Q.append(xk[0,0]/np.pi*180)
    I.append(ik[0,0])
    xk = A*xk + B*ik

plt.plot(time, Q, label='pendulum angle, radians')
plt.plot(time, I, label='control current, ampers')

plt.legend(loc='upper right')
plt.grid()
plt.show()

