#!/usr/bin/python3

import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

def build_parameters(r):
    v = 0.020*np.pi*r*r # volume
    lr = .5
    mr = 0.020+v*2698.9
    lp = .35
    mp = .350
    Jp = .005
    return {'g':9.81, 'k':0.0369, 'R':0.608, 'U':24., 'Jr':0.0000181 + mr*r*r/2, 'J':Jp + mp*lp*lp + mr*lr*lr, 'ml':mp*lp + mr*lr}

def simulate_linear(parameters, x0, time):
    ml = parameters['ml']
    J  = parameters['J']
    Jr = parameters['Jr']
    g  = parameters['g']
    k  = parameters['k']
    R  = parameters['R']
    U  = parameters['U']

    A = np.matrix([[0.,1.,0.],[ml*g/J, 0., k**2/(J*R)],[-ml*g/J, 0., -(J+Jr)/(J*Jr)*k**2/R]])
    B = np.matrix([[0.],[-k/(J*R)],[(J+Jr)/(J*Jr)*k/R]])

    synthetic = [x0.copy().transpose().tolist()[0]]
    xi = x0.copy()
    for i in range(len(time)-1):
        xi = xi + (A*xi+B*U)*(time[i+1]-time[i])
        synthetic.append(xi.copy().transpose().tolist()[0])
    return np.transpose(synthetic)

def derivative(parameters, state):
    ml = parameters['ml']
    J  = parameters['J']
    Jr = parameters['Jr']
    g  = parameters['g']
    k  = parameters['k']
    R  = parameters['R']
    U  = parameters['U']
    [theta, dtheta, dthetar] = state
    return np.array([dtheta, -k*U/(J*R)+k**2/(J*R)*dthetar + ml*g/J*np.sin(theta), (J+Jr)/(J*Jr)*(k*U/R - k**2/R*dthetar) - ml*g/J*theta])

def simulate_nonlinear(parameters, x0, time):
    synthetic = [x0.copy().transpose().tolist()[0]]
    xi = synthetic[-1][:]
    for i in range(len(time)-1):
        h = time[i+1]-time[i]
        k1 = derivative(parameters, xi)*h
        k2 = derivative(parameters, xi+k1/2.)*h
        k3 = derivative(parameters, xi+k2/2.)*h
        k4 = derivative(parameters, xi+k3)*h
        xi = xi+k1/6.+k2/3.+k3/3.+k4/6.

#        d = derivative(parameters, xi)
#        xi = [sum(x) for x in zip(xi, [a*(time[i+1]-time[i]) for a in d])]
        synthetic.append(xi[:])
    return np.transpose(synthetic)

def plot_max_linear(maxr):
    gr = []
    gx = []
    bestx = -1
    bestr = -1

    for r in np.arange(0., maxr, 0.001):
        parameters = build_parameters(r)

        ml = parameters['ml']
        J  = parameters['J']
        Jr = parameters['Jr']
        g  = parameters['g']
        k  = parameters['k']
        R  = parameters['R']
        U  = parameters['U']

        A = np.matrix([[0.,1.,0.],[ml*g/J, 0., k**2/(J*R)],[-ml*g/J, 0., -(J+Jr)/(J*Jr)*k**2/R]])
        B = np.matrix([[0.],[-k/(J*R)],[(J+Jr)/(J*Jr)*k/R]])

        [eigenvalues, eigenvectors] = np.linalg.eig(A)
        assert eigenvalues[0]>0 and eigenvalues[1]<0 and eigenvalues[2]<0, "Houston we've got a problem"

        V = np.matrix(eigenvectors)
        ymax = (V.I*B).item(0,0)*U/eigenvalues[0]
        xmax = np.abs(ymax/V.I.item(0,0))
        if (bestx<xmax):
            bestr = r
            bestx = xmax
        gr.append(r*2.)
        gx.append(xmax)
    return [gr, gx]

def plot_max_nonlinear(maxr):
    gr = []
    gx = []

    time = np.linspace(0, 2, 5000)

    for r in np.arange(0., maxr, 0.001):
        print(r)
        parameters = build_parameters(r)
        xmax = -1
        a = .0
        b = .5
        simu_nonlinear_a = simulate_nonlinear(parameters, np.matrix([a, 0., 0.]).transpose(), time)
        simu_nonlinear_b = simulate_nonlinear(parameters, np.matrix([b, 0., 0.]).transpose(), time)
        while True:
            simu_nonlinear_m = simulate_nonlinear(parameters, np.matrix([(a+b)/2., 0., 0.]).transpose(), time)

            if simu_nonlinear_m[0,-1]<0:
                a = (a+b)/2.
                simu_nonlinear_a = simu_nonlinear_m
            else:
                b = (a+b)/2.
                simu_nonlinear_b = simu_nonlinear_m

            if (np.abs(a-b)<1e-5):
                break
        xmax = a
        gr.append(r*2.)
        gx.append(xmax)
    return [gr, gx]


[grl, gxl] = plot_max_linear(.2)
[grn, gxn] = plot_max_nonlinear(.2)
fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Max possible angle to achieve a stabilization")
ax1.set_ylabel('angle, rad')
ax1.set_xlabel('reaction wheel diameter, m (aluminium disk, 20mm thickness)') 
ax1.plot(grl,gxl,color='red', label='linear simulation')
ax1.plot(grn,gxn,color='blue', label='nonlinear simulation')
ax1.legend()
plt.show()


'''
fig = plt.figure()
ax1 = fig.add_subplot(111)
ax1.set_title("Pendulum dynamics")
ax1.set_xlabel('Time, sec')

x0 = np.matrix([0.2430166, 0., 0.]).transpose() # theta thetadot
time = np.linspace(0, 2, 2000)
parameters = build_parameters(.057)
simu_nonlinear = simulate_nonlinear(parameters, x0, time)
simu_linear    = simulate_linear   (parameters, x0, time)

ax1.plot(time, simu_nonlinear[0,:], label='angle, rad (nonlinear simu)')
ax1.plot(time, simu_linear   [0,:], label='angle, rad (linear simu)')
ax1.legend()
plt.show()
'''

