'''
FIlE demo-inchworm.py
This file helps us control a 5 segment robot using two methods
1. starting a gaitCyclePlan from a CSV file on user input 
2. by using axes of a joystick to control induvidual motors
'''
from joy.decl import *
from joy import JoyApp, StickFilter
from joy.misc import curry
from joy.plans import GaitCyclePlan
from joy.misc import loadCSV
class InchwormApp( JoyApp ):
  '''
  This is a concrete class which controls a 5 segment robot in two modes using a joystick and 
  CSV file 
  
  It uses the joystick's front buttons to toggle between the modes in onEvent
  1. joystick mode:
  It uses a stickfilter object to take joystick input and uses setIntegrator function to filter
  the input. And then it uses set_pos function on the module respective to the joystick axis used.
  2. gait Mode: 
  It takes a CSV file created by demo-makeInchwormCSV.py to implement a GainCyclePlan and starts 
  and controls the plan using keyboard input in onEvent function

  It would be useful for those who want control a robot with various motors using joystick 
  or CSV file  
  '''
  # Load the gait table for the inchworming gait
  # NOTE: we could equally well generate the gait table here by pasting in the
  #   code from 'demo-makeInchwormCSV.py', but this would not demonstrate the
  #   use of .csv files, which can be edited with a spreadsheet
  SHEET = loadCSV("inchworm.csv")
  
  def __init__(self,*arg,**kw):
    # We expect a 5 module robot
    JoyApp.__init__(self,robot=dict(count=5),*arg,**kw)
    
  def onStart( self ):
    # Construct the inchworm gait Plan 
    gait = GaitCyclePlan( self, self.SHEET, maxFreq = 1)
    gait.setFrequency(0.2)
    # We don't strictly need the following, but it's nice to be able to see
    #   an explicit message when the GaitCyclePlan starts and stops. For such
    #   simple changes we don't need a sub-class -- we can overwrite the 
    #   methods with our own functions at runtime.
    # Using curry(...) allows us to dynamically create a function that consists 
    #   of a call to progress(...) with a pre-set string. See joy.curry for
    #   details; this is a staple "trick" from functional programming.
    gait.onStart = curry(progress,">>> START")
    gait.onStop = curry(progress,">>> STOP")
    self.gait = gait
    #
    # We set up integrators for the direct joystick control mode, using the 
    #   StickFilter Plan as an interface component    
    sf = StickFilter(self)
    sf.setIntegrator( 'joy0axis0',10 ,lower=-9,upper=9)
    sf.setIntegrator( 'joy0axis1',10 ,lower=-9,upper=9)
    sf.setIntegrator( 'joy0axis2', 10 ,lower=-9,upper=9)
    sf.setIntegrator( 'joy0axis3', 10 ,lower=-9,upper=9)
    sf.setIntegrator( 'joy0axis4', 10 ,lower=-9,upper=9)
    sf.start()
    self.sf = sf
    # Start with joystick mode disabled
    self.joyMode = False
    # Create a temporal filter that is true once every 0.5 seconds
    self.oncePer = self.onceEvery(0.5)
  
  def onStop( self ):
    # Make sure we go_slack on all servo modules when we exit
    self.robot.off()
  
  def onEvent(self, evt):
    ### Main event handler
    # We have several broad classes of events, which are processed differently
    #  depending on our mode. In most cases, we return after handling the event.
    #
    # Allow operator to quit by closing window or hitting 'q' or <escape>
    #  (actually, the superclass call at the end of this method would do all
    #  of this, but it is good practice to clean our own mess if we can)
    if evt.type==QUIT or (evt.type==KEYDOWN and evt.key in [K_q,K_ESCAPE]):
      self.stop()
    #
    # Joystick buttons
    if evt.type==JOYBUTTONDOWN:
      # Buttons on the "front" of the controller toggle mode
      if evt.button>3:
        self.joyMode = not self.joyMode
        if self.joyMode:
          if self.gait.isRunning():
            self.gait.stop()
          progress("*"*20 + " joystick mode ON")
        else:
          progress("*"*20 + " joystick mode OFF")
          self.robot.off()
        return
      else: # else buttons on the controller face --> PANIC and go slack
        progress("!"*20 + " RESET: joystick OFF, gait STOPPED, modules SLACK")
        if self.gait.isRunning():
          self.gait.stop()        
        self.joyMode = False
        self.robot.off()
        return True        
    #
    # If in joystick mode --> punt to onEvent_joyMode
    if self.joyMode:
      if self.onEvent_joyMode(evt):
        return
    else: # else in gait mode --> punt to onEvent_gaitMode
      if self.onEvent_gaitMode(evt):
        return
    #
    # We reach this point if no previous handler processed the event.
    #   If it is not a TIMEREVENT, we use the superclass onEvent handler
    #   which prints a human readable description of the event object.
    JoyApp.onEvent(self,evt)

  def onEvent_joyMode( self, evt ):
    # Joystick motions are pushed to the stick-filter
    if evt.type==JOYAXISMOTION:
      self.sf.push(evt)
      return True
    if evt.type==TIMEREVENT :
      lClaw = self.sf.getValue("joy0axis1")
      lAng = self.sf.getValue("joy0axis0")
      rClaw = self.sf.getValue("joy0axis3")
      rAng = self.sf.getValue("joy0axis2")
      body = self.sf.getValue("joy0axis4")
      self.robot.at.w1.set_pos(lClaw * 1000)
      self.robot.at.w2.set_pos(lAng * 1000)
      self.robot.at.w3.set_pos(body * 1000)
      self.robot.at.w4.set_pos(rAng * 1000)
      self.robot.at.w5.set_pos(rClaw * 1000) 
      if self.oncePer():
        progress("<<< %g %g < %g > %g %g >>>" % (lClaw,lAng,body,rClaw,rAng) )
      return True

  def onEvent_gaitMode(self, evt ):
    # All gait control happens via the keyboard; non KEYDOWN events are ignored
    if evt.type!=KEYDOWN:
      return False
    #
    # [enter] toggles gait on and off
    if evt.key==K_KP_ENTER:
      if self.gait.isRunning():
        self.gait.stop()
        progress( "Gait stopped" )
        return True
      else:
        self.gait.start()
        progress( "Gait starting" )
        return True
    #
    # [space] freezes cycling in place
    if evt.key==K_SPACE: 
      self.gait.setFrequency(0)
      progress('Stopped motion')
      return True
    # 
    # [up] and [down] change frequency
    if evt.key in (K_UP,K_DOWN):
      f = self.gait.getFrequency()
      # Change frequency up/down in range -limit..limit hz
      if evt.key==K_DOWN:
        f -= 0.03
      else:
        f += 0.03
      self.gait.setFrequency(f)
      progress('Frequency changed to %.2f Hz' % self.gait.getFrequency())
      return True
    #
    # Digits jump to relative positions in the cycle
    if evt.key in range(K_0,K_9+1):
      self.gait.setFrequency(0)
      phi = ('1234567890'.find(chr(evt.key)))/9.0
      self.gait.moveToPhase( phi )
      progress('Moved to phase %.2f' % phi)
      return True
      
if __name__=="__main__":
  print ("""
  Inchworm Robot Demo
  -------------------
  
  This demo combines several JoyApp capabilities to give a full-fleged
  application for controlling a 5-segment worm-like robot.
  
  The demo uses a gait stored in 'inchworm.csv', which can be generated
  by piping the output of demo-makeInchwormCSV.py, as follows:
  
  $ python demos/demo-makeInchwormCSV.py > demos/inchworm.csv
  
  To make this work with your own robot, make sure to name your modules
  w1,w2,w3,w4 and w5 in order (the nodeNames entry in JoyApp.yml), and to
  make sure that the line:
        signs = array([1,-1,1,1,-1])
  is changed to reflect the rotation directions of your modules.
  
  Operation
  ---------
  When the demo runs, the operation can switch between two modes: the "joystick"
  mode and the "gait" mode. In joystick mode, each of the first 5 game
  controller axes controls a robot DOF directly. In gait mode, the keyboard 
  controls starting, stopping, stepping and jumping around in the gait cycle.
  
  In addition, the first 4 game controller buttons are "panic" buttons that 
  reset the robot to a known state.
  
  All other controller buttons toggle between joystick and gait modes.
  
  In gait mode, the following keys are used:
    [up] / [down] -- control frequency
    [space] -- freeze in place
    [1]...[9],[0] -- jump to a relative location in the cycle [1] start, [0] end
    [enter] -- toggle gait on and off.
  """)
  app=InchwormApp()
  app.run()

