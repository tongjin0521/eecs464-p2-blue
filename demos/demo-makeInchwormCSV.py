from numpy import *
import sys

def inchworm(phi):
  """Programmatic definition of inchworm gait"""
  crv = 0.25
  ass = 0.35
  crv0 = 0.1
  W =array([0, -0.75,-2,0,2,0.75])
  N = len(W)
  if phi<0.33:
    a = (phi-0.33/2)*6
    res = (-ones(N)*pi*crv - W*a*ass)
  elif phi>=0.33 and phi<0.66:
    a = -(phi-0.5)*6
    b = 1-(phi-0.33)*3    
    res = (-ones(N)*pi*((crv-crv0)*b+crv0) - W*b*ass)
  elif phi>=0.66:
    a = -(phi-(1.66/2))*6
    b = (phi-0.66)*3    
    res = (-ones(N)*pi*((crv-crv0)*b+crv0) + W*b*ass)    
  return res

if __name__!="__main__":
  raise "Don't run me"

# Our modules are oriented in the [1,-1,1,1,-1] directions relative to each
#   other but we've found that we want the first and last modules to
#   counter-rotate to get the gait right.
signs = array([1,-1,1,1,-1])
sg = -signs
sg[0] *= -1
sg[-1] *= -1

# Create the CSV file headings
sys.stdout.write('"t","w1","w2","w3","w4","w5"\n')
# Sample 300 points, without the final point (since 1 is the same as 0)
for phi in linspace(0,1,30,endpoint=False):
  s = inchworm(phi)
  line = ("%.3g," % phi) + ",".join( 
    [ "%d" % (18000.0/pi*val) for val in s[1:]*sg ] )
  sys.stdout.write(line+"\n")

sys.stderr.write("""
This program is a quick hack for creating a gait table for an inchworm gait.  
To use this program, pipe its output into a .csv file
""")
