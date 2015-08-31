# tab seperated values parser
# created for BIRDS Lab at the University of Michigan
# for use with floquet analysis tools by
# Shai Revzen Phd.
#
# Chad Schaffer
# cmschaf@umich.edu
# 5/7/2015
from numpy import *
import os

direct = './data'
data = []

for root, dirs, files in os.walk(direct):
        for file in files:
            name = direct + '/' + file
            data.append(genfromtxt(name, delimiter='\t', skip_header=12))


print data


