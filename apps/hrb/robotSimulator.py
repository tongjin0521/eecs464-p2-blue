# file robotSimulator.py simulates a robot in an arena

from sensorPlanTCP import SensorPlanTCP
from robotSimIX import SimpleRobotSim,RobotSimInterface
from joy import JoyApp, progress
from joy.decl import *
from joy.plans import Plan
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )
from pylab import randn,dot,mean,exp,newaxis

class MoveForward(Plan):
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 10
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in xrange(self.N):
      s.move(step)
      yield self.forDuration(dt)

class Turn(Plan):
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Angle to turn [rad]
    self.ang = 0.1
    # Duration of travel [sec]
    self.dur = 3.0
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute rotation step
    dt = self.dur / float(self.N)
    step = self.ang / float(self.N)
    for k in xrange(self.N):
      s.turn(step)
      yield self.forDuration(dt)


class RobotSimulatorApp( JoyApp ):
  """Concrete class RobotSimulatorApp <<singleton>>
     A JoyApp which runs the DummyRobotSim robot model in simulation, and
     emits regular simulated tagStreamer message to the desired waypoint host.

     Used in conjection with waypointServer.py to provide a complete simulation
     environment for Project 1
  """
  def __init__(self,wphAddr=WAYPOINT_HOST,*arg,**kw):
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml", *arg, **kw
      )
    self.srvAddr = (wphAddr, APRIL_DATA_PORT)

  def onStart( self ):
    # Set up socket for emitting fake tag messages
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
    s.bind(("",0))
    self.sock = s
    # Set up the sensor receiver plan
    self.sensor = SensorPlanTCP(self,server=self.srvAddr[0])
    self.sensor.start()
    self.robSim = SimpleRobotSim(fn=None)
    self.moveP = MoveForward(self,self.robSim)
    self.turnP = Turn(self,self.robSim)
    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1/20.0)
    progress("Using %s:%d as the waypoint host" % self.srvAddr)
    self.T0 = self.now

  def showSensors( self ):
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
    else:
      progress( "Waypoints: << no reading >>" )

  def emitTagMessage( self ):
    """Generate and emit and update simulated tagStreamer message"""
    self.robSim.refreshState()
    # Get the simulated tag message
    msg = self.robSim.getTagMsg()
    # Send message to waypointServer "as if" we were tagStreamer
    self.sock.sendto(msg, self.srvAddr)

  def onEvent( self, evt ):
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus():
      self.showSensors()
      progress( self.robSim.logLaserValue(self.now) )
      # generate simulated laser readings
    elif self.timeForLaser():
      self.robSim.logLaserValue(self.now)
    # update the robot and simulate the tagStreamer
    if self.timeForFrame():
      self.emitTagMessage()

    if evt.type == KEYDOWN:
      if evt.key == K_UP and not self.moveP.isRunning():
        self.moveP.dist = 100.0
        self.moveP.start()
        return progress("(say) Move forward")
      elif evt.key == K_DOWN and not self.moveP.isRunning():
        self.moveP.dist = -100.0
        self.moveP.start()
        return progress("(say) Move back")
      if evt.key == K_LEFT and not self.turnP.isRunning():
        self.turnP.ang = 0.5
        self.turnP.start()
        return progress("(say) Turn left")
      if evt.key == K_RIGHT and not self.turnP.isRunning():
        self.turnP.ang = -0.5
        self.turnP.start()
        return progress("(say) Turn right")
    # Use superclass to show any other events
      else:
        return JoyApp.onEvent(self,evt)
    return # ignoring non-KEYDOWN events

if __name__=="__main__":
  print """
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer.
  """
  import sys
  if len(sys.argv)>1:
      app=RobotSimulatorApp(wphAddr=sys.argv[1], cfg={'windowSize' : [160,120]})
  else:
      app=RobotSimulatorApp(wphAddr=WAYPOINT_HOST, cfg={'windowSize' : [160,120]})
  app.run()
