# -*- coding: utf-8 -*-
"""Evaluates a data run. Must pass argument for data file name and initial conditions (gait param file)"""


import string
import os
from optutil import fminIter
import string
import numpy
import re
from decimal import Decimal


def evaluate(trialnum):
        """Convert a results file to a score"""
        fname = "result"+str(trialnum)
        dname = "dist" + str(trialnum)
        if os.path.exists(fname) and os.path.exists(dname):
            with open(fname, 'r') as f:
                first = f.readline()
                last = list(f)[-1]
                time = float(last) - float(first)
    
            with open(dname, 'r') as f:
                dist = f.readline()


            return time / float(dist)  
        else:
            print "File does not exist."
            print fname
            return