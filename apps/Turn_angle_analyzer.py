# takes data from a .CSV file and returns the angle of the turn
#
# this program was written for data taken from the Mechapod robot
# at BIRDS Lab University of Michigan
#
# 6/6/2015
#
# Chad Schaffer
# cmschaf@umich.edu
import pylab
import numpy as np
from scipy import signal as S
import util as U


x = pylab.loadtxt('/home/your-grace/Downloads/2000twist_40spd_1_pos.tsv', delimiter=',',skiprows=13)
x.shape = (x.shape[0], x.shape[1]/3, 3)

A,B = S.butter(3, 0.01)
xf = pylab.empty_like(x)

for k in xrange(x.shape[1]):
    for kk in xrange(x.shape[2]):
        xf[:,k,kk] = S.filtfilt(A,B,x[:,k,kk])



zf = xf[...,0]+1j*xf[...,1]

R_tot = []
I_tot = []

for n in xrange(zf.shape[0]):
    R_tot.append(pylab.mean(zf[n,...].real))
    I_tot.append(pylab.mean(zf[n,...].imag))

c = np.argmax(R_tot)

R_for_tot = R_tot[0:c]
I_for_tot = I_tot[0:c]

R_back_tot = R_tot[c:-1]
I_back_tot = I_tot[c:-1]

for_arc = np.empty([len(R_back_tot), 2])



for n in xrange(len(R_back_tot)):
    for_arc[n][0] = R_back_tot[n]
    for_arc[n][1] = I_back_tot[n]

#with np.errstate(divide='ignore', invalid='ignore'):
elip = U.fit_ellipse(for_arc)

#elip -> [0]a, [1]b, [2]c, [3]d, [4]e, [5]f

h = (-elip[4]*elip[1] + 2 * elip[2] * elip[3])/ (elip[1]**2 - 4 * elip[0] * elip[2])

k = (-elip[3]*elip[1] + 2 * elip[0] * elip[4])/ (elip[1]**2 - 4 * elip[0] * elip[2]) 


R_for_tot = R_for_tot - h

I_for_tot = I_for_tot - k 


pylab.plot(R_for_tot, I_for_tot)

pylab.show()



