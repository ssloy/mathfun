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


g = 9.81

m = 2.62

Ic = 0.001
Jc = 0.000260
Kc = 0.000987

Id = 0.000564
Jd = 0.0011
Kd = 0.000564

Ib = 2*0.01045
Jb = 2*0.01312
Kb = 2*0.01358

Ia = 2*0.01045
Ja = 2*0.01312
Ka = 2*0.01358

wd1 =  223.6
wd2 = -223.6

l = .1344

A = np.matrix([[0.,1.,0.,0.],[g*l*m/(l**2*m +2*Kc+Jb+2*Id),0.,0.,0.],[0,0,0,0],[0,0,0,0]])
B = np.matrix([[0.,0.], [Jd*wd1/(l**2*m +2*Kc+Jb+2*Id), Jd*wd2/(l**2*m +2*Kc+Jb+2*Id)], [1.,0.], [0.,1.]])

print("A=", A)
print("B=", B)

Q = np.matrix('''
100.  0.    0.   0.;
  0. 10.    0.   0.;
  0.  0.  100.   0.;
  0.  0.    0. 100. ''')

R = np.matrix("100. 0.; 0. 100.")

print("Q=", Q)
print("R=", R)

K = lqr(A,B,Q,R)
np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)
print("K=", K)

