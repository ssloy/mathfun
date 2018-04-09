#!/usr/bin/python
from scipy import misc
import numpy as np

rotor = misc.imread('brake-rotor-203mm-1.9mm-200g.png')
assert rotor.shape == (1024,1024,4), "Houston we've got a problem"

J = np.float64(0.)
cnt = 0
for x in range(1024):
    for y in range(1024):
        if rotor[x,y,0]<128:
            J = J + (np.sqrt((x-512)**2 + (y-512)**2)*.203/1024.)**2
            cnt = cnt+1
J = J*.2/cnt

print("brake rotor inertia", J)

hub_mass = .015*.03**2*np.pi*1380
hub_inertia = .03**2/2 * hub_mass

print ("pvc cylinder 15mm x 60mm diameter, moment of inertia", hub_inertia)
print ("pvc cylinder 15mm x 60mm diameter, mass", hub_mass)

screws_mass = .040
screws_inertia = screws_mass*.022**2

print("screws mass", screws_mass)
print("screws inertia", screws_inertia)

rotor_mass = .050 # those are listed in the datasheet for the motor
rotor_inertia = .0000181

print("total mass", .203+hub_mass    + screws_mass    + rotor_mass)
print("total inertia", J+hub_inertia + screws_inertia + rotor_inertia)

