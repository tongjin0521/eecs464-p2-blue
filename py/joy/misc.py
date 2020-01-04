#  This library is free software; you can redistribute it and/or
#  modify it and use it for any purpose you wish.
#
#  The library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
# (c) Shai Revzen, U Penn, 2010, U Michigan 2019, 2020
#
"""
misc.py defines several useful functions that don't rightly belong in any obvious location

Functions
---------

curry -- Curry a function, returning a callable with the value of the initial parameters pre-selected. One of the mainstays of functional programming.

printExc -- Print a formatted stack-trace to standard error

loadCSV -- Load a CSV file into a list of lists

inlineCSV -- Similar to loadCSV, but takes the CSV text from a multiline python string
"""

from traceback import format_exception
from sys import stderr, exc_info, argv
from pdb import set_trace as BRK
from os import getenv

from joy.pygix import SCHD
def requiresPyGame():
    """
    Check whether running in pygame mode, and if not, raise an appropriate
    RuntimeError giving the user instructions how to restart the process with
    pygame enabled.
    """
    #it extracts the sink destination port from the arguement and deletes it thereafter
    if SCHD == "pygame":
        return
    raise RuntimeError("\n"+"*"*40+"\nRemote source should not be used without activating the pygame scheduler. Try something like: 'PYGIXSCHD=pygame python %s' on your commandline" % argv[0])

def curry(fn, *cargs, **ckwargs):
  """
  Curry a function, returning a callable with the value of the
  initial parameters pre-selected. For any function f
    curry(f,*arg0)(*arg1,**kw) f(x,*(arg0+arg1),**kw)
  so, for example:
    f(x,y) = curry(f,x)(y) = curry(f,x,y)()
  """
  def call_fn(*fargs, **fkwargs):
      d = ckwargs.copy()
      d.update(fkwargs)
      return fn(*(cargs + fargs), **d)
  return call_fn

def printExc( exinfo=None ):
  """
  Print a formatted stack-trace to standard error
  """
  # Print the stack trace so far to stderr
  if exinfo is None:
    exinfo = exc_info()
  msg = ["<< "*10+"\n"]
  msg.extend(format_exception(*exinfo))
  msg.append(">> "*10+"\n")
  stderr.writelines(msg)
  if 'B' in getenv("JOYDEBUG",""):
      BRK()

def loadCSV( fn ):
  """
  Load a CSV file into a list of lists, converting empty cells into
  None, numbers as floats and quoted strings as strings.

  In addition to standard CSV format, empty (white only) lines and
  lines starting with "#" are ignored
  """
  if type(fn)==str:
    f = open(fn,'r')
  else:
    assert iter(fn),"Must be iterable returning lines"
    f = fn
  def convert(x):
    x = x.strip()
    if not x or x=='""': return None
    try:
      return float(x)
    except ValueError:
      if x[0] in {'"',"'"}:
        return x[1:-1]
      else:
        return x
  res = [
    [ convert(x) for x in l.split(",") ]
    for l in f if l.strip() and l[:1]!="#"
  ]
  if type(fn)==str:
    f.close()
  return res

def inlineCSV( txt ):
  """
  Similar to loadCSV, but takes the CSV text from a multiline
  python string, e.g.

  >>> sheet = inlineCSV('''
  ... # Non-CSV Unix shell style comments and empty lines are ignored
  ... ,         "sparrow",  "banana"
  ... "weight", 5,          1
  ... ''')
  """
  return loadCSV(( l for l in txt.split("\n") ))
