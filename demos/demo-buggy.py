'''
FILE demo-buggy.py

This file is used to move two servos by receiving inputs via a Joystick
It would be useful for those who want to control motors using joystick or keyboard
'''
# We will create our own plan subclass and use stickfilter to make joystick input easy
from joy import (Plan, JoyApp, StickFilter)
# boilerplate import from joyapp, always use this
from joy.decl import *

class Buggy( Plan ):
  '''
  Concrete class buggy

  This class creates a plan to move the motors while receiving joystick values
  when joystick motion is detected

  It receives input through the getValue function of the stickfilter which its parent
  class starts and moves the motors through set_torque function

  Expects its owner Joyapp to have:
  1. An .sf StickFilter which can be used to obtain input
  2. A robot with .lwheel and .rwheel modules in motor mode
  With those in place, just start() and stop() the Buggy Plan to start  and stop the motors
  '''
  def __init__(self,*arg,**kw):
    Plan.__init__(self,*arg,**kw)
    self.r = self.app.robot.at

  # Start message
  def onStart( self ):
    progress("Buggy started")

  def onStop( self ):
    """End message"""
    progress("Stopping")
    # this code makes sure a right wheel and a left wheel are set to zero speed
    self.r.lwheel.set_torque(0)
    self.r.rwheel.set_torque(0)
  #acts on our inputs
  def behavior( self ):
    oncePer = self.app.onceEvery(0.5)
    while True:
      yield
      # Read joystick from application's StickFilter
      sf = self.app.sf
      # the front-back values of the left and right joysticks
      lspeed = sf.getValue('joy0axis1')
      rspeed = sf.getValue('joy0axis2')
      #progress("SPD %d %d" % ( lspeed,rspeed))

      self.r.lwheel.set_torque(lspeed)
      self.r.rwheel.set_torque(rspeed)

      # prints out the status of the wheels every .5 seconds
      if oncePer():
        progress("Buggy: left wheel %6f right wheel %6f"
          % (lspeed,rspeed))

class BuggyApp( JoyApp ):
  '''
  This is a concrete ByggyApp which starts/stops the above plan plan on Joystick input

  It does that in onEvent function
  '''


  '''
  This searches for two modules named 'front' and 'rear'
  in the /pyckbot/cfg/joyapp.yml file
  '''
  def __init__(self,robot=dict(count=2),*arg,**kw):
    #declares an empty dictionary
    cfg = dict ()
    # initializes the application's app.robot attribute
    JoyApp.__init__(self,robot=robot,cfg=cfg,*arg,**kw)

  def onStart(self):
    # we need stickFilter functionality to connect to the joystick
    # dt means the time step for the filter. .5 represents 20 timers per sec
    sf = StickFilter(self,dt=0.05)
    # tells the stickfilter that it is using joystick's joy0axis1 and joy0axis2
    # and smoothens those signals using the loss pas filter with a cutoff time of
    # 10 samples and a time constant of 0.5 secons
    sf.setLowpass( 'joy0axis1',10)
    sf.setLowpass( 'joy0axis2',10)
    sf.start()
    # store the plan instance in the buggyApp .sf attribute
    self.sf = sf
    # create a buggy plan instance and store it in .ma
    self.ma = Buggy(self)
    self.robot.at.lwheel.set_mode("Motor")
    self.robot.at.rwheel.set_mode("Motor")

  def onEvent(self,evt):
    if evt.type == JOYAXISMOTION:
      if evt.joy == 0 and evt.axis == 2 and evt.value == 0:
        return # Silently drop buggy controller events
      # Forward a copy of the event to the StickFilter plan
      self.sf.push(evt)
    # Buttons --> press button5 to start buggy; release to stop
    elif evt.type==JOYBUTTONDOWN and evt.joy==0 and evt.button==5:
      self.ma.start()
    elif evt.type==JOYBUTTONUP and evt.joy==0 and evt.button==5:
      self.ma.stop()

    # Hide robot position events
    elif evt.type==CKBOTPOSITION:
      return
    JoyApp.onEvent(self,evt)

  def onStop(self):
    self.ma.onStop()


#main function
if __name__=="__main__":
  #creates an interface object and starts the interface
  app = BuggyApp()
  #directs to the onStart function and starts a plan which creates thread
  app.run()
