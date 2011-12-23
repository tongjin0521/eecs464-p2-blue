"""
Helper script for plotting buehler clock

"""

from rapidQuad import Buehler
import numpy
import pylab

t_start = 0
t_end = 1.01
t_interval = 0.01

b = Buehler()
t = numpy.arange(t_start, t_end, t_interval)
ph = map(b.buehler, t)
pylab.plot(t,ph)
pylab.show()

