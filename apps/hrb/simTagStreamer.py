# file simTagStreamer.py simulates a robot in an arena

from sensorPlan import SensorPlan
from robotSim import DummyRobotSim
from joy import JoyApp, progress
from joy.decl import *
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT

class RobotSimulatorApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml",
      ) 

  def onStart( self ):
    # Set up the sensor receiver plan
    self.sensor = SensorPlan(self)
    self.sensor.start()
    self.robSim = DummyRobotSim(fn=None)
    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1/20.0)

  def showSensors( self ):
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %d f %d b %d" % (ts,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %d " % ts + str(w))
    else:
      progress( "Waypoints: << no reading >>" )
  
  def emitTagMessage( self ):
    """Generate and emit and update simulated tagStreamer message"""
    self.robSim.refreshState()
    # Get the simulated tag message
    msg = self.robSim.getTagMsg()
    # Use the sensor socket (dual-use) to send to waypointServer
    self.sensor.sendto(msg, (WAYPOINT_HOST, APRIL_DATA_PORT))
    
  def onEvent( self, evt ):
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus(): 
      self.showSensors()
    # generate simulated laser readings
    if self.timeForLaser():
      progress( self.robSim.logLaserValue(self.now) )
    # update the robot and simulate the tagStreamer
    if self.timeForFrame(): 
      self.emitTagMessage()

    if evt.type == KEYDOWN:
      if evt.key == K_UP:
        self.robSim.move(0.5)
        return progress("(say) Move forward")
      elif evt.key == K_DOWN:
        self.robSim.move(-0.5)
        return progress("(say) Move back")
      elif evt.key == K_LEFT:
        self.robSim.turn(-0.5)
        return progress("(say) Turn left")
      elif evt.key == K_RIGHT:
        self.robSim.turn(0.5)
        return progress("(say) Turn right")
    # Use superclass to show any other events
    return JoyApp.onEvent(self,evt)


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
  app=RobotSimulatorApp()
  app.run()

