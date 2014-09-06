# -*- coding: utf-8 -*-
"""
Created on Thu Sep  4 20:31:13 2014

@author: shrevzen-home
"""
from json import dumps as json_dumps
from numpy import asfarray, dot
from numpy.random import randn
from waypointShared import *

MSG_TEMPLATE = {
       0: [[502, 251], [479, 272], [508, 296], [530, 274]],
       1: [[469, 347], [471, 311], [434, 305], [431, 339]],
       2: [[466, 226], [437, 209], [420, 231], [450, 248]],
       3: [[362, 204], [339, 224], [365, 245], [387, 224]],
       4: [[334, 336], [370, 326], [359, 294], [324, 301]],
       22: [[425, 145], [424, 169], [455, 173], [455, 149]],
       23: [[556, 302], [594, 308], [588, 277], [553, 272]],
       24: [[284, 258], [294, 229], [261, 221], [250, 249]],
       25: [[432, 424], [431, 384], [391, 380], [392, 419]],
       26: [[559, 399], [564, 440], [606, 444], [599, 403]],
       27: [[546, 163], [546, 187], [578, 193], [576, 170]],
       28: [[212, 397], [253, 395], [254, 355], [214, 358]],
       29: [[319, 129], [290, 124], [283, 145], [313, 153]]
  }
  
def tags2list( dic ):
    """ 
    Convert a dictionary of tags into part of a list for JSON serialization
    
    INPUT:
      dic -- dictionary mapping tag id to 4x2 corner location array
      
    OUTPUT:
      list to be concatenated into JSON message
    """
    return [ 
        { 
          u'i' : k, 
          u'p': [ list(row) for row in v ] 
        } 
        for k,v in dic.iteritems()
    ]

def findXing(a,b):
  """
  Find the crossing point of two lines, represented each by a pair of
  points on the line
  
  INPUT:
    a -- 2x2 -- two points, in rows
    b -- 2x2 -- two points, in rows
  
  OUTPUT: c -- 2 -- a point, as an array
  """
  a = asfarray(a)
  b = asfarray(b)
  # The nullspace of this matrix is the projective representation
  # of the intersection of the lines. Each column's nullspace is
  # one of the lines
  X = concatenate([a[1]-a[0],b[0]-b[1],a[0]-b[0]],0)
  Q = svd(X)[0]
  # Last singular vector is basis for nullspace; convert back from
  # projective to Cartesian representation
  q = Q[:2,2]/Q[2,2]
  c = q[0]*(a[1]-a[0])+a[0]
  return c
  
class RobotSimInterface( object ):
  """
  Abstract superclass RobotSimInterface defines the output-facing interface
  of a robot simulation.
  
  Subclasses of this class must implement all of the methods
  """
  def __init__(self, lln=None):
    """
    INPUT:
      lln -- filename / None -- laser log name to use for logging simulated 
          laser data. None logged if name is None
          
    ATTRIBUTES:
      tagPos -- 4x2 float array -- corners of robot tag
      laserAxis -- 2x2 float array -- two points along axis of laser
      waypoints -- dict -- maps waypoint tag numbers to 4x2 float 
          arrays of the tag corners
    """
    self.tagPos = asfarray(MSG_TEMPLATE[ ROBOT_TAGID[0]])
    self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
    self.waypoints = { tid : asfarray(MSG_TEMPLATE[tid]) for tid in waypoints }
    self._msg = None
    
  def refreshState( self ):
    """<<pure>> refresh the value of self.tagPos and self.laserAxis"""
    print "<<< MUST IMPLEMENT THIS METHOD >>>"
    
  def getTagMsg( self ):
    """
    Using the current state, generate a TagStreamer message simulating
    the robot state
    """
    # Cache a list of the corner tags. They don't move
    if self._msg is None:
      self._msg = tags2list({ tid : MSG_TEMPLATE[tid] for tid in corners})
    # Collect robot and waypoint locations
    state = { ROBOT_TAGID[0] : self.tagPos }
    state.update( self.waypoints )
    # Combine all into a list
    msg = tags2list(state) + self._msg
    # Serialize
    return json_dumps(msg)

  def getLaserValue( self ):
    """
    Using the current state, generate a fictitious laser pointer reading
    """
    <<< TODO >>>
    
class DummyRobotSim( RobotSimInterface ):
  def __init__(self):
    RobotSimInterface.__init__(self)
    self.dNoise = 0.1
    self.aNoise = 0.1
    
  def move( self, dist ):
    """
    Move forward some distance
    """
    # Compute a vector along the forward direction
    fwd = dot([[1,-1,1,-1],[1,-1,1,-1]],self.tagPos)/2
    # Move all tag corners forward by distance, with some noise
    self.tagPos = self.tagPos + fwd[newaxis,:] * dist * (1+randn()*self.dNoise)

  def turn( self, angle ):
    """
    Turn by some angle
    """
    z = dot(self.tagPos,[1,1j])
    c = mean(z)
    zr = c + (z-c) * exp(1j * (angle+randn()*self.aNoise)
    self.tagPos[:,0] = zr.real
    self.tagPos[:,1] = zr.imag
    
  def refreshState( self ):
    """
    Make state ready for use by client.
    
    ALGORITHM:
    Since the entire robot state is captured by the location of the
    robot tag corners, update the laser axis from the robot tag location 
    """
    self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
    
