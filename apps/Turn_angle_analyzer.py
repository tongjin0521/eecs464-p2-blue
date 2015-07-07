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

#zf -= 1000-1200j

total = pylab.angle(pylab.mean(zf * pylab.exp(1j*pylab.angle(zf).min()),1))

c = 0

for n in zf[0]:
    if zf[n+1][0] < zf[n][0]:
        c = n
        break


print c
#el = pylab.asfarray([U.fit_ellipse(xf[400:,k,:2]) for k in xrange(xf.shape[1])])

#mEl = pylab.mean(el,0)

#el.shape = el.shape[:2]

#zc = el[:,3]/el[:,0]+1j*el[:,4]/el[:,2]

pylab.plot(zf)

pylab.show()



