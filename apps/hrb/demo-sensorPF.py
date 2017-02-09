"""
FILE: demo-sensorPF.py

  Demonstration of how to use a particle filter to process line sensor 
  responses
"""

# Go headless
from matplotlib import use as mpl_use
mpl_use('Agg')
frm1 = "./dpf/fr1-%02d.png"
frm2 = "./dpf/fr2-%02d.png"
print "Output frames at:",frm1, frm2
fr = 0

from time import sleep
from waypointShared import lineSensorResponse, lineDist

# Number of particles
Num = 50
# Step size
stepSize = 0.02
# Sensor Noise 
sNoi = 0.001
# Movement Noise
mNoi = 0.01
# Min percentile for keeping particle
minP = 25
  
# Locations
pts = rand(Num)+1j*rand(Num)
# Weights
wgt = ones(Num)
# Line
abLine = asarray([.2+0.5j,.8+0.7j])
# Actual position
pos = 0.5+0.5j

while True:
  ### "robot" simulation
  # Move "robot"
  step = (randn() + randn()*1j) * stepSize
  pos += step
  # Get "robot" sensor measurement
  dpos = lineDist([pos],abLine[0],abLine[1])
  spos = lineSensorResponse(dpos, sNoi)
  
  ### Particle filter
  # Move particles
  pts += step + (randn(Num)+1j*randn(Num)) * mNoi
  # Get particle sensor measurements
  d = lineDist(pts,abLine[0],abLine[1])
  s = lineSensorResponse(d, sNoi)
  
  # Adjust weights, keeping matching sensors with heigher weight
  #   We penalize sensor mismatch
  #   We penalize all high weights. Because average weight is reset to 1.0,
  #     this implies that in absence of 
  #wgt = 1.0+abs(spos-s)*0.5+wgt*0.1
  wgt = wgt * 0.6 + 0.4 / (1.0+abs(spos-s)*0.5)
  wgt = Num * wgt / sum(wgt)
  
  # Find best particle
  best = argmax(wgt)

  # Replace low weight particles
  idx = find(wgt<prctile(wgt,minP))
  if any(idx):
    m = len(idx)
    pts[idx] = pts[best]
    wgt[idx] = 1.0
  

  if 1: # draw graphics  
    figure(1); clf()
    title('weights')
    plot( abLine.real, abLine.imag, 'o-k', lw=3 )
    scatter(pts.real, pts.imag, s=10+wgt*wgt*4, c=linspace(0,1,Num), alpha=0.5 )
    plot( pos.real, pos.imag, 'r+', ms = 15, mew=3)
    plot( pts[best].real, pts[best].imag, 'bx', ms = 15, mew=2)
    axis('equal'); axis([-0.5,1.5,-0.5,1.5])
    savefig(frm1 % fr)
    figure(2); clf()    
    bar(arange(Num),wgt)
    savefig(frm2 % fr)
    fr = (fr + 1) % 100
    sleep(0.05)