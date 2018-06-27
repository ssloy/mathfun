#!/usr/bin/python3

import sys
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from lmfit import minimize, Minimizer, Parameters, Parameter, report_fit
from functools import reduce

def derivative(params, state, amps): 
    [Q, dQ, Qr, dQr] = state
    
    g = 9.81
    k = 0.0369
    Jr = 0.001248
    mr = .35
    lr = .22

    pdict = params.valuesdict()
    Jp = pdict['Jp']
    mp = pdict['mp']
    lp = pdict['lp']

    J = Jp + mr*lr*lr + mp*lp*lp
    ml = mp*lp + mr*lr

    A = np.matrix([[J+Jr, Jr],[Jr, Jr]])
    B = np.matrix([[ml*g*np.sin(Q)],[k*amps]])
    C = inv(A)*B
    return np.array([dQ, C[0,0], dQr, C[1,0]])


def rk4_step(params, yi, i, time, current): 
    h = time[i+1]-time[i]
    k1 = derivative(params, yi, current[i])*h
    k2 = derivative(params, yi+k1/2., (current[i]+current[i+1])/2.)*h
    k3 = derivative(params, yi+k2/2., (current[i]+current[i+1])/2.)*h
    k4 = derivative(params, yi+k3, current[i+1])*h
    return yi+k1/6.+k2/3.+k3/3.+k4/6.

def predict(params, time, current):
    sol = []
    yi = [-3.1416, 0., 0., 0.]
    for i in range(len(time)-1):
        yi = rk4_step(params, yi, i, time, current)
        sol.append(yi)
    sol.append(yi)
    return np.transpose(sol)
    
def energy(params, data):
    E = 0.
    for run in data:
        y = predict(params, run[0], run[1])
        for i in range(len(run[2])):
            E = E + (y[0,i]-run[2][i])**2 #+ .001*(y[2,i]-run[3][i])**2
    print(params.valuesdict(),"\n", E)
    return E

def build_params():
    Jp = 0.0038
    mp = 0.578
    lp = 0.10


    params = Parameters()
    params.add('Jp', value=Jp, min=.0001, max=.1)
    params.add('mp', value=mp, min=.3, max=1.)
    params.add('lp', value=lp, min=.05, max=.2)

    return params


params = build_params()

runs = []
runs = runs + ["chirp-a.csv"]
#runs = runs + ["chirp-b.csv"]

data = []
for run in runs:
    [time, ref, I, Qr, Q] = np.loadtxt(run, delimiter=',', skiprows=1, unpack=True)
#    data.append([time[:3000], I[:3000], Q[:3000], Qr[:3000]])
    data.append([time, I, Q, Qr])


minner = Minimizer(energy, params, fcn_args=[data])
if (0):
    result = minner.minimize(method='powell')
    report_fit(result)
    params=result.params

#ax1.plot(time, x,      color='blue', label='cart position, m')
#ax1.plot(time, current,  color='cyan', label='cart speed, m/s')
#ax1.plot(time, refcurrent,  color='cyan', label='cart speed, m/s')

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum parameters fitting")
ax1.set_xlabel('Time, sec')

energy(params, data)
for run in data:
    synthetic = predict(params, run[0], run[1])
#    ax1.plot(run[0], run[1],  label='current')
    ax1.plot(run[0], run[2],  label='pendulum angle, rad')
    ax1.plot(run[0], synthetic[0,:], label='synthetic pendulum angle')
    ax1.plot(run[0], run[3],  label='RW angle, rad')
    ax1.plot(run[0], synthetic[2,:], label='synthetic RW angle')


ax1.legend()
plt.show()
