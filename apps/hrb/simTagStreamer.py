# file simTagStreamer.py simulates a robot in an arena
from sys import stdout
from time import time as now
from struct import pack
# Uses UDP sockets to communicate
from socket import (
  socket, AF_INET,SOCK_DGRAM, SOCK_STREAM, inet_aton, IPPROTO_UDP, 
  INADDR_ANY, IPPROTO_IP, IP_ADD_MEMBERSHIP, error as SocketError
  )
# Packets are JSON enocoded
from json import loads as json_loads, dumps as json_dumps
# Uses numpy arrays for the math
from numpy import (
  array,asarray,zeros, exp,sign,linspace, uint8,
  zeros_like,kron, pi, empty_like, nan, isnan,
  concatenate,newaxis, mean, median, dot, inf
  )
from numpy.random import rand, randn
# Visualizes the state using matplotlib 2D figures
from pylab import figure, subplot, gcf, plot, axis, text, find, draw
# Include all the modeling functions provided to the teams
#  this ensures that what the server does is consistent with the model given
#  to students during the development process
from waypointShared import *
# The main program is a JoyApp
from joy import JoyApp, Plan

class SimTagPlan( Plan ):
  def __init__( self, app, robotSim, *arg, **kw ):
    Plan.__init__(self, app, *arg, **kw )
    self.sock = None
    self.lastSensorReading = None
  def behavior( self ):
    q = self.app.remote
    # Allow sensor data in via the RemoteSink
    q.flushMisc()
    while True:
      for t,dic in q.queueIter():
        dic = dic.items()
        dic.sort()
        progress("Received update:")
        for k,v in dic:
          progress("   %s : %s" % (k,repr(v)))
      yield self.forDuration(0.3)

class SensorPlan( Plan ):
  """
 import socket
import struct

MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5007

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', MCAST_PORT))
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

while True:
  print sock.recv(10240)
  """
  """
  SensorPlan is a concrete Plan subclass that uses a UDP socket to 
  and decode WayPoint messages
  """
  def __init__( self, app, *arg, **kw ):
    Plan.__init__(self, app, *arg, **kw )
    self.sock = None
    self.lastSensorReading = None
          
  def _connect( self ):
    """Set up the socket"""
    s = socket(AF_INET, SOCK_STREAM, IPPROTO_UDP)
    mreq = pack("4sl", inet_aton(WAYPOINT_LISTENER_GROUP), INADDR_ANY)
    s.setsockopt(IPPROTO_IP, IP_ADD_MEMBERSHIP, mreq)
    s.bind("0.0.0.0",WAYPOINT_MSG_PORT)
    s.setblocking(0)
    self.sock = s

  def stop( self ):
    """(called when stopping) clean up the socket, if there is one"""
    if self.sock is not None:
      self.sock.close()
    self.sock = None

  def ensureConnection( self ):
    """(sub-behavior) loops until the socket is up"""
    while True:
      # if not connected --> try to connect
      if self.sock is None:
        self._connect()
      # if not connected --> sleep for a bit
      if self.sock is None:
        yield self.forDuration(0.1)
      else: # otherwise --> done, return to parent 
        return 
  
  def _nextMessage( self ):
        """
        Obtain the next message; kill socket on error. 
        
        returns '' if nothing was received
        """
        # receive an update / skip
        try:
          return self.sock.recv(1024)
        except SocketError, se:
          # If there was no data on the socket
          #   --> not a real error, else kill socket and start a new one
          if se.errno != 11:
            progress("Connection failed: "+str(se))
            self.sock.close()
            self.sock = None
        return '' # nothing received
        
  def behavior( self ):
    """
    Plan main loop    
    """
    while True:
      # If no socket set up --> activate the ensureConnection sub-behavior to fix
      if self.sock is None:
        yield self.ensureConnection()
      msg = self._nextMessage()
      # If no message --> sleep a little and try again
      if len(msg) is 0:
          yield self.forDuration(0.3)
          continue
      # Parse the message
      dic = json_loads(msg)
      ts = self.app.now
      self.lastSensorReading = (ts, dic['f'], dic['b'])
      # Make sure to allow others to get the CPU
      yield

class WaypointSensorApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml",
      ) 

  def onStart( self ):
    # Set up the sensor receiver plan
    self.sensor = SensorPlan(self)
    self.sensor.start()

  def onEvent( self, evt ):
    # Punt to superclass
    # this is here to remind you to override it
    return JoyApp.onEvent(evt)


if __name__=="__main__":
  print """
  Running the waypoint sensor demo

  Listens on local port 0xBAA (2986) for incoming waypoint sensor
  information.

  The waypoint sensor send JSON maps with keys:
  'f', 'b' : front and back sensor values
  'w' : list of lists. Each sub-list is of length 2. List of waypoint
    coordinates, including the next waypoint. Each time the next
    waypoint changes, it means the previous waypoint was reached.
  """
  app=WaypointSensorApp()
  app.run()

