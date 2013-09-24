"""Evaluates a data run. Must pass argument for data file name and initial conditions (gait param file)"""

import string
import os
from optutil import fminIter
import string
import numpy


def evaluate(trialnum):
    """Convert a results file to a score"""
    fname = "result"+str(trialnum)
    if os.path.exists(fname):
        fin = open(fname,'r')
        return(-trialnum) #Placeholder for the code that computes a score, score gets lower as trials go up		
    else:
        print "File does not exist."
        print fname
        return