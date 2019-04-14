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

A = np.matrix([[0,1,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,1,0,0,0,0],[0,(np.sqrt(2)*Jd*wd2+np.sqrt(2)*Jd*wd1)/(2*l**2*m+4*Kc+2*Jb+4*Id),(g*l*m)/(l**2*m+2*Kc+Jb+2*Id),0,0,(Jd*wd1)/(l**2*m+2*Kc+Jb+2*Id),0,(Jd*wd2)/(l**2*m+2*Kc+Jb+2*Id)],[0,0,0,0,0,1,0,0],[0,0,0,-(Jd*wd1)/(Id+Ic),0,0,0,0],[0,0,0,0,0,0,0,1],[0,0,0,-(Jd*wd2)/(Id+Ic),0,0,0,0]])
B = np.matrix([[0,0],[-2/(np.sqrt(2)*Kb+2**(3/2)*Ka+2**(3/2)*Jd+2**(3/2)*Jc+np.sqrt(2)*Ib),-2/(np.sqrt(2)*Kb+2**(3/2)*Ka+2**(3/2)*Jd+2**(3/2)*Jc+np.sqrt(2)*Ib)],[0,0],[0,0],[0,0],[(Kb+2*Ka+2*Jd+2*Jc+Id+Ic+Ib)/((Id+Ic)*Kb+(2*Id+2*Ic)*Ka+(2*Id+2*Ic)*Jd+(2*Id+2*Ic)*Jc+Ib*Id+Ib*Ic),1/(Kb+2*Ka+2*Jd+2*Jc+Ib)],[0,0],[1/(Kb+2*Ka+2*Jd+2*Jc+Ib),(Kb+2*Ka+2*Jd+2*Jc+Id+Ic+Ib)/((Id+Ic)*Kb+(2*Id+2*Ic)*Ka+(2*Id+2*Ic)*Jd+(2*Id+2*Ic)*Jc+Ib*Id+Ib*Ic)]])

print("A=", A)
print("B=", B)

Q = np.matrix('''
 100.  0.    0.  0.   0.  0.  0.  0.;
  0.  .1    0.  0.   0.  0.  0.  0.;
  0.  0.  100.  0.   0.  0.  0.  0.;
  0.  0.    0. 10.   0.  0.  0.  0.;
  0.  0.    0.  0.  10.  0.  0.  0.;
  0.  0.    0.  0.   0.  1.  0.  0.;
  0.  0.    0.  0.   0.  0. 10.  0.;
  0.  0.    0.  0.   0.  0.  0.  1. ''')

R = np.matrix("20. 0.; 0. 20.")

print("Q=", Q)
print("R=", R)

K = lqr(A,B,Q,R)
print("K=", K);

dt = .001
qa = .5
wa = 0.
qb = -np.pi/4.+.2
wb = 0.
qc1 = np.pi/2.
wc1 = 0.
qc2 = np.pi/2.
wc2 = 0.


QA = []
QB = []
QC1 = []
QC2 = []
T = []

nsteps = 10000
time = np.linspace(0, nsteps*dt, nsteps, endpoint=True)

for t in time:
    Tau = -K*np.matrix([[qa],[wa],[qb+np.pi/4],[wb],[qc1-np.pi/2],[wc1],[qc2-np.pi/2],[wc2]])
    M = np.matrix([[(Kc-Jd-Jc+Id)*np.cos(qb)**2*np.cos(qc2)**2+(Kc-Jd-Jc+Id)*np.cos(qb)**2*np.cos(qc1)**2+(Kb+2*Jd+2*Jc-2*Id-2*Ic-Ib)*np.cos(qb)**2+Ka+2*Id+2*Ic+Ib,((-Kc)+Jd+Jc-Id)*np.cos(qb)*np.cos(qc2)*np.sin(qc2)+((-Kc)+Jd+Jc-Id)*np.cos(qb)*np.cos(qc1)*np.sin(qc1),((-Id)-Ic)*np.sin(qb),((-Id)-Ic)*np.sin(qb)],[((-Kc)+Jd+Jc-Id)*np.cos(qb)*np.cos(qc2)*np.sin(qc2)+((-Kc)+Jd+Jc-Id)*np.cos(qb)*np.cos(qc1)*np.sin(qc1),(Kc-Jd-Jc+Id)*np.sin(qc2)**2+(Kc-Jd-Jc+Id)*np.sin(qc1)**2+l**2*m+2*Jd+2*Jc+Jb,0,0],[((-Id)-Ic)*np.sin(qb),0,Id+Ic,0],[((-Id)-Ic)*np.sin(qb),0,0,Id+Ic]])
    C = np.matrix([[(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)**2*np.cos(qc2)*np.sin(qc2)*wc2+((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)**2*np.cos(qc1)*np.sin(qc1)*wc1+(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.sin(qb)*np.cos(qc2)**2+((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.sin(qb)*np.cos(qc1)**2+((-2*Kb)-4*Jd-4*Jc+4*Id+4*Ic+2*Ib)*np.cos(qb)*np.sin(qb))*wb)/2,((-Jd*np.sin(qb)*np.sin(qc2)*wd2)-Jd*np.sin(qb)*np.sin(qc1)*wd1+(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.cos(qc2)**2+(Kc-Jd-Jc-Ic)*np.cos(qb))*wc2+(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.cos(qc1)**2+(Kc-Jd-Jc-Ic)*np.cos(qb))*wc1+((2*Kc-2*Jd-2*Jc+2*Id)*np.sin(qb)*np.cos(qc2)*np.sin(qc2)+(2*Kc-2*Jd-2*Jc+2*Id)*np.sin(qb)*np.cos(qc1)*np.sin(qc1))*wb+(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.sin(qb)*np.cos(qc2)**2+((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.sin(qb)*np.cos(qc1)**2+((-2*Kb)-4*Jd-4*Jc+4*Id+4*Ic+2*Ib)*np.cos(qb)*np.sin(qb))*wa)/2,(Jd*np.cos(qb)*np.cos(qc1)*wd1+(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.cos(qc1)**2+(Kc-Jd-Jc-Ic)*np.cos(qb))*wb+((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)**2*np.cos(qc1)*np.sin(qc1)*wa)/2,(Jd*np.cos(qb)*np.cos(qc2)*wd2+(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.cos(qc2)**2+(Kc-Jd-Jc-Ic)*np.cos(qb))*wb+((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)**2*np.cos(qc2)*np.sin(qc2)*wa)/2],[(Jd*np.sin(qb)*np.sin(qc2)*wd2+Jd*np.sin(qb)*np.sin(qc1)*wd1+((2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qb)*np.sin(qc2)**2+((-Kc)+Jd+Jc+Ic)*np.cos(qb))*wc2+((2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qb)*np.sin(qc1)**2+((-Kc)+Jd+Jc+Ic)*np.cos(qb))*wc1+(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.sin(qb)*np.sin(qc2)**2+((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.sin(qb)*np.sin(qc1)**2+(4*Kc+2*Kb-4*Ic-2*Ib)*np.cos(qb)*np.sin(qb))*wa)/2,((2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qc2)*np.sin(qc2)*wc2+(2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qc1)*np.sin(qc1)*wc1)/2,((-Jd*np.sin(qc1)*wd1)+(2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qc1)*np.sin(qc1)*wb+((2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qb)*np.sin(qc1)**2+((-Kc)+Jd+Jc+Ic)*np.cos(qb))*wa)/2,((-Jd*np.sin(qc2)*wd2)+(2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qc2)*np.sin(qc2)*wb+((2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qb)*np.sin(qc2)**2+((-Kc)+Jd+Jc+Ic)*np.cos(qb))*wa)/2],[((-Jd*np.cos(qb)*np.cos(qc1)*wd1)+(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.sin(qc1)**2+(Kc-Jd-Jc-Ic)*np.cos(qb))*wb+(2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qb)**2*np.cos(qc1)*np.sin(qc1)*wa)/2,(Jd*np.sin(qc1)*wd1+((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qc1)*np.sin(qc1)*wb+(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.sin(qc1)**2+(Kc-Jd-Jc-Ic)*np.cos(qb))*wa)/2,0,0],[((-Jd*np.cos(qb)*np.cos(qc2)*wd2)+(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.sin(qc2)**2+(Kc-Jd-Jc-Ic)*np.cos(qb))*wb+(2*Kc-2*Jd-2*Jc+2*Id)*np.cos(qb)**2*np.cos(qc2)*np.sin(qc2)*wa)/2,(Jd*np.sin(qc2)*wd2+((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qc2)*np.sin(qc2)*wb+(((-2*Kc)+2*Jd+2*Jc-2*Id)*np.cos(qb)*np.sin(qc2)**2+(Kc-Jd-Jc-Ic)*np.cos(qb))*wa)/2,0,0]])
    G = np.matrix([[((Jd*np.cos(qb)*np.cos(qc2)*wc2-Jd*np.sin(qb)*np.sin(qc2)*wb)*wd2+(Jd*np.cos(qb)*np.cos(qc1)*wc1-Jd*np.sin(qb)*np.sin(qc1)*wb)*wd1)/2],[-((Jd*np.sin(qc2)*wc2-Jd*np.sin(qb)*np.sin(qc2)*wa)*wd2+(Jd*np.sin(qc1)*wc1-Jd*np.sin(qb)*np.sin(qc1)*wa)*wd1+2*g*l*m*np.sin((4*qb+np.pi)/4))/2],[((Jd*np.sin(qc1)*wb-Jd*np.cos(qb)*np.cos(qc1)*wa)*wd1)/2],[((Jd*np.sin(qc2)*wb-Jd*np.cos(qb)*np.cos(qc2)*wa)*wd2)/2]])
#   print("M=", M)
#   print("C=", C)
#   print("G=", G)

    ddQ = M.I*(np.matrix([[0.],[0.],[Tau[0,0]],[Tau[1,0]]]) - C*np.matrix([[wa], [wb], [wc1], [wc2]]) - G)

    wa = wa + dt*ddQ[0,0]
    wb = wb + dt*ddQ[1,0]
    wc1 = wc1 + dt*ddQ[2,0]
    wc2 = wc2 + dt*ddQ[3,0]

    qa = qa + dt*wa
    qb = qb + dt*wb
    qc1 = qc1 + dt*wc1
    qc2 = qc2 + dt*wc2

    QA.append(qa)
    QB.append(qb)
    QC1.append(qc1)
    QC2.append(qc2)
    T.append(t)
    if (np.abs(qb+np.pi/4)>.3):# or (np.abs(qc-np.pi/3)>.16):
        break

plt.plot(T, QA, label='qa, rad')
plt.plot(T, QB, label='qb, rad')
plt.plot(T, QC1, label='qc1, rad')
plt.plot(T, QC2, label='qc2, rad')
#plt.plot(T,  U, label='control torque, Nm')

plt.legend(loc='upper right')
plt.grid()
plt.show()
