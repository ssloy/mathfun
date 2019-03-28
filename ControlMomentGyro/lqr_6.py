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


A = np.matrix([[0.0,1.0,0.0,0.0,0.0,0.0],[0.0,-2.326461567185816,0.1239984448448164,0.0,0.0,-3.290113500653945],[0.0,0.0,0.0,1.0,0.0,0.0],[0.0,2.422610372270755,55.76149433910259,0.0,0.0,3.426088444811039],[0.0,0.0,0.0,0.0,0.0,1.0],[0.0,57.09687689900345,-0.08768014120635591,-135.8286646728406,0.0,2.326461567185816]]);
B = np.matrix([[0.0],[-26.88739596601124],[0.0],[-0.02538234092892752],[0.0],[658.3984492743275]]);

print("A=", A)
print("B=", B)


Q = np.matrix('''
.1 0.   0.  0.  0.  0.;
 0. 0.01   0.  0.  0.  0.;
 0. 0. 1000.  0.  0.  0.;
 0. 0.   0. 10.  0.  0.;
 0. 0.   0.  0. 10.  0.;
 0. 0.   0.  0.  0. 10.''')

R = np.matrix("100.")

print("Q=", Q)
print("R=", R)

#print(np.transpose(B))

K = lqr(A,B,Q,R)
print("double K[] = {%f, %f, %f, %f, %f, %f};" % (K[0,0], K[0,1], K[0,2], K[0,3], K[0,4], K[0,5]))

dt = .0001
qa = 0.1
qb = -np.pi/4.+0.1
qc =  np.pi/3.#+0.1
wa = 0.
wb = 0.
wc = 0.


QA = []
QB = []
QC = []
T = []

nsteps = 30000
time = np.linspace(0, nsteps*dt, nsteps, endpoint=True)

for t in time:
#    tauc = -(K[0,0]*qa + K[0,1]*wa + K[0,2]*(qb+np.pi/4) + K[0,3]*wb + K[0,4]*(qc-np.pi/3.) + K[0,5]*wc)
    K = [4.800283, 0.502054, -0.316228, 0.123239]
    tauc = (K[0]*(qb+np.pi/4) + K[1]*wb + K[2]*(qc-np.pi/3.) + K[3]*wc)

    M = np.matrix([[(-1.909999999999999E-4*np.cos(qb)**2*np.sin(qc)**2)+0.003117*np.cos(qb)**2+0.025594,-1.909999999999999E-4*np.cos(qb)*np.cos(qc)*np.sin(qc),-0.001564*np.sin(qb)],[-1.909999999999999E-4*np.cos(qb)*np.cos(qc)*np.sin(qc),1.909999999999999E-4*np.sin(qc)**2+0.06180600319999999,0.0],[-0.001564*np.sin(qb),0.0,0.001564]]);
    C = np.matrix([[(-1.909999999999999E-4*np.cos(qb)**2*np.cos(qc)*np.sin(qc)*wc)+1.909999999999999E-4*np.cos(qb)*np.sin(qb)*np.sin(qc)**2*wb-0.003117*np.cos(qb)*np.sin(qb)*wb,-0.5*((0.001754999999999999*np.cos(qb)-3.819999999999999E-4*np.cos(qb)*np.sin(qc)**2)*wc-3.819999999999999E-4*np.sin(qb)*np.cos(qc)*np.sin(qc)*wb-3.819999999999999E-4*np.cos(qb)*np.sin(qb)*np.sin(qc)**2*wa+0.006234*np.cos(qb)*np.sin(qb)*wa+0.2453*np.sin(qb)*np.sin(qc)),0.5*(3.819999999999999E-4*np.cos(qb)*np.sin(qc)**2*wb-0.001754999999999999*np.cos(qb)*wb-3.819999999999999E-4*np.cos(qb)**2*np.cos(qc)*np.sin(qc)*wa+0.2453*np.cos(qb)*np.cos(qc))],[0.5*((3.819999999999999E-4*np.cos(qb)*np.sin(qc)**2+0.001373*np.cos(qb))*wc-3.819999999999999E-4*np.cos(qb)*np.sin(qb)*np.sin(qc)**2*wa+0.006234*np.cos(qb)*np.sin(qb)*wa+0.2453*np.sin(qb)*np.sin(qc)),1.909999999999999E-4*np.cos(qc)*np.sin(qc)*wc,-0.5*((-3.819999999999999E-4*np.cos(qc)*np.sin(qc)*wb)-3.819999999999999E-4*np.cos(qb)*np.sin(qc)**2*wa-0.001373*np.cos(qb)*wa+0.2453*np.sin(qc))],[-0.5*(3.819999999999999E-4*np.cos(qb)*np.sin(qc)**2*wb+0.001373*np.cos(qb)*wb-3.819999999999999E-4*np.cos(qb)**2*np.cos(qc)*np.sin(qc)*wa+0.2453*np.cos(qb)*np.cos(qc)),0.5*((-3.819999999999999E-4*np.cos(qc)*np.sin(qc)*wb)-3.819999999999999E-4*np.cos(qb)*np.sin(qc)**2*wa-0.001373*np.cos(qb)*wa+0.2453*np.sin(qc)),0.0]])
    G = np.matrix([[111.5*(0.0011*np.cos(qb)*np.cos(qc)*wc-0.0011*np.sin(qb)*np.sin(qc)*wb)],[-0.5*(223.0*(0.0011*np.sin(qc)*wc-0.0011*np.sin(qb)*np.sin(qc)*wa)+6.90875136*np.sin(0.25*(4.0*qb+3.141592653589793)))],[111.5*(0.0011*np.sin(qc)*wb-0.0011*np.cos(qb)*np.cos(qc)*wa)]])
#   print("M=", M)
#   print("C=", C)
#   print("G=", G)


    ddQ = M.I*(np.matrix([[0.],[0.],[tauc]]) - C*np.matrix([[wa], [wb], [wc]]) - G)
    wa = wa + dt*ddQ[0,0]
    wb = wb + dt*ddQ[1,0]
    wc = wc + dt*ddQ[2,0]
    qa = qa + dt*wa
    qb = qb + dt*wb
    qc = qc + dt*wc
    QA.append(qa)
    QB.append(qb)
    QC.append(qc)
    T.append(t)
    if (np.abs(qb+np.pi/4)>.2):# or (np.abs(qc-np.pi/3)>.16):
        break


'''
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
'''

#plt.plot(time, errQB, label='~ qb')
#plt.plot(time, errdQB, label='~ wb')
#plt.plot(time, errQC, label='~ qc')
#plt.plot(time, errdQC, label='~ wc')

#plt.plot(time, hatQB, label='HAT pendulum angle, radians')
#plt.plot(time, hatQC, label='HAT gyro angle, radians')
#plt.plot(time, hatdQB, label='^wb')
#plt.plot(time, hatdQC, label='^wc')
#
plt.plot(T, QA, label='A orientation, rad')
plt.plot(T, QB, label='pendulum angle, rad')
plt.plot(T, QC, label='gyro angle, rad')
#plt.plot(time, I, label='control current, A')

plt.legend(loc='upper right')
plt.grid()
plt.show()

