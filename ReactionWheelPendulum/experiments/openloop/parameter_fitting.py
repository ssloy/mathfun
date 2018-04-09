#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from lmfit import minimize, Minimizer, Parameters, Parameter, report_fit


k  = 0.0369
Jr = 0.001388
Cr = 0.00001
I  = .5

def predict(params, time, current):
    globals().update(params.valuesdict())
    sol = [ t*k*I/Cr + k*I*Jr/Cr**2*np.exp(-t*Cr/Jr) - k*I*Jr/Cr**2 for t in time]
    return sol

def energy(params, data):
    E = 0.
    for run in data:
        y = predict(params, run[0], run[1])
        for i in range(len(run[2])):
            E = E + (y[i]-run[2][i])**2
    print(params.valuesdict(),"\n", E)
    return E


params = Parameters()
#params.add('k',  value=0.0369,   min=.0001, max=.5)
params.add('Jr', value=0.001388, min=.0001, max=.5)
params.add('Cr', value=0.00001,  min=.0001, max=.5)


runs = []
runs = runs + ['vise-constant-0.5a.csv']

data = []
for run in runs:
    [time, refcurrent, current, Qr, Q] = np.loadtxt('vise-constant-0.5a.csv', delimiter=',', skiprows=1, unpack=True)
    data.append([time, current, Qr])

minner = Minimizer(energy, params, fcn_args=[data])
result = minner.minimize(method='powell')
report_fit(result)
params=result.params

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Pendulum parameters fitting")
ax1.set_xlabel('Time, sec')

energy(params, data)
for run in data:
    synthetic = predict(params, run[0], run[1])
    ax1.plot(run[0], run[2],   label='RW angle, rad')
    ax1.plot(run[0], synthetic, label='synthetic angle')

ax1.legend()
plt.show()

