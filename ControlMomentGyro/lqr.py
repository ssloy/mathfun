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
k = 1.62 # Nm/A

J1 = Jd + Jb + Kc + m*l*l
J2 = Jc - Id + Jd - Kc
m0 = m*g*l/np.sqrt(2)

A = np.matrix([[0., 1., 0., 0.], [m*g*l/J1, 0., 0., Jd*wd/J1], [0., 0., 0., 1.], [0., -Jd*wd/(Ic+Id), 0., 0.]])
B = np.matrix([[0.],[0.],[0.],[1./(Ic+Id)]])
Q = np.matrix("100. 0. 0. 0.; 0. 10. 0. 0. ; 0. 0. 10. 0.; 0. 0. 0. 1. ")
R = np.matrix("100.")


#print(J1,J2)
print("A:", A)
print("B:", B)
#print(lqr(A, B, Q, R))

C = np.matrix("1. 0. 0. 0. ; 0. 0. 1. 0.")
print("C:", C)
K = lqr(A,B,Q,R)
print("double K[] = {%f, %f, %f, %f};" % (K[0,0], K[0,1], K[0,2], K[0,3]))

nsteps = 3000
time = np.linspace(0, nsteps*dt, nsteps, endpoint=True)

QB  = []
QC  = []
I  = []

qb = -np.pi/4. + .2
qc = np.pi/2.
wb = 0.
wc = 0.

hatX = np.matrix([[0.],[0.],[0.],[0]])

#L = np.matrix([[30000., -.05],[5000., -1500.],[-.05, 30000.],[-1700.,190000]])
#L = np.matrix([[ 0.0218  , 0.0093 ], [  -2.8527 ,  0.2257], [  -0.3243 ,  0.0242], [  -7.4431 , -3.2165]])


#P = [-50., -51., -52., -153.]
P = [-50., -51., -52., -153.]
fsf = signal.place_poles(np.transpose(A), np.transpose(C), P, method='YT')
L = np.transpose(fsf.gain_matrix)
print(L)

hatdQB = []
hatdQC = []
hatQB = []
hatQC = []
errdQB = []
errdQC = []
errQB = []
errQC = []

for t in time:
    tauc = 0 #-(K[0,0]*(qb+np.pi/4) + K[0,1]*wb + K[0,2]*(qc-np.pi/2) + K[0,3]*wc)
    if (t>0):
        tauc = -(K[0,0]*(qb+np.pi/4) + K[0,1]*hatdQB[-1] + K[0,2]*(qc-np.pi/2) + K[0,3]*hatdQC[-1])

    dwb = (J2*np.sin(qc*2)*wc*wb + Jd*np.sin(qc)*wc*wd + m0*(np.sin(qb)+np.cos(qb)))/(J1+J2*(np.cos(qc)**2))
    dwc = (tauc - Jd*np.sin(qc)*wb*wd - .5*J2*np.sin(2*qc)*(wb**2))/(Ic+Id)
    wb = wb + dt*dwb
    wc = wc + dt*dwc
    qb = qb + dt*wb
    qc = qc + dt*wc

    dhatX = A*hatX + B*tauc + L*(np.matrix([[qb+np.pi/4],[qc-np.pi/2]]) - C*hatX)
    hatX = hatX + dhatX*dt

    hatdQB.append(hatX[1,0])
    hatdQC.append(hatX[3,0])

    hatQB.append(hatX[0,0]-np.pi/4.)
    hatQC.append(hatX[2,0]+np.pi/2.)
    QB.append(qb)
    QC.append(qc)
    I.append(tauc/k)
    errQB.append(hatQB[-1]-QB[-1])
    errdQB.append(hatX[1,0] - wb)
    errQC.append(hatQC[-1]-QC[-1])
    errdQC.append(hatX[3,0] - wc)

#plt.plot(time, errQB, label='~ qb')
#plt.plot(time, errdQB, label='~ wb')
#plt.plot(time, errQC, label='~ qc')
#plt.plot(time, errdQC, label='~ wc')

#plt.plot(time, hatQB, label='HAT pendulum angle, radians')
#plt.plot(time, hatQC, label='HAT gyro angle, radians')
#plt.plot(time, hatdQB, label='^wb')
#plt.plot(time, hatdQC, label='^wc')
#
plt.plot(time, QB, label='pendulum angle, radians')
plt.plot(time, QC, label='gyro angle, radians')
plt.plot(time, I, label='control current, A')

plt.legend(loc='upper right')
plt.grid()
plt.show()

