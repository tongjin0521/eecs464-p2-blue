'''
FILE demo-arm.py

This file demonstrates the movement of robotic arm and gripper using 5 motors and a joystick. It does that by calculating the position
of each servo when user gives the skew and bend values and bringing the motors rto that position using set_pos function
'''
from joy.decl import *
from joy import JoyApp, Plan, Stckfilter

class MoveArm( Plan ):
  '''
  This is a concrete class which creates a plan for arm movement based on joystick input

  It calculates the angles to be set based on bend and skew given by joystick input. It receives input through the getValue function of the
  stickfilter which its owner JoyApp class had created. It also  calculates wrist and grip movement values based on user input. It implements
  these calculations on a robot by setting the motors' position using set_pos function

  Expects its owner Joyapp to have:
  1. An .sf StickFilter which can be used to obtain input
  2. A robot with 5 modules arm1, arm2, arm3, arm4, grip in '0' mode
  This will be useful for those who want to make a robotic arm and observe its movements.
  '''
  def __init__(self,*arg,**kw):
    Plan.__init__(self,*arg,**kw)
    r = self.app.robot.at
    self.seg = [r.arm1,  r.arm3, r.arm4, r.arm5] # r.arm2,
    self.grip = r.grip

  def onStart( self ):
    progress("Arm motion started")

  def onStop( self ):
    progress("Going slack")
    for mod in [self.grip]+self.seg:
      mod.go_slack()
    progress("Arm motion stopped")

  def behavior( self ):
    oncePer = self.app.onceEvery(0.5)
    while True:
      yield
      # Read joystick from application's StickFilter
      sf = self.app.sf
      bend = sf.getValue('joy0axis0')
      skew = sf.getValue('joy0axis1')
      wrist = sf.getValue('joy0axis4')
      grip = abs(sf.getValue('joy0axis3')) # must be positive!
      #
      # Set arm angles
      ang = [(bend+skew*k)*self.app.cfg.armGain for k in range(-len(self.seg)/2,len(self.seg)/2+1)]
      # Set angles of all except wrist
      for mod,pos in zip(self.seg[:-1],ang[:-1]):
        mod.set_pos(pos)
      # Set wrist angle
      w = ang[-1]+wrist*self.app.cfg.wristGain
      self.seg[-1].set_pos(w)
      # Set gripper
      self.grip.set_pos(grip*self.app.cfg.gripGain)
      #
      if oncePer():
        progress("Arm: bend %6f skew %6f wrist %6f grip %6f"
          % (bend,skew,wrist,grip))

class ArmApp( JoyApp ):
  '''
  This class implements the above plan on joystick input using start and stop functions

  This does that in onEvent
  '''
  def __init__(self,robot=dict(count=5),*arg,**kw):
    #initialize an arm with default gain vlaues
    cfg = dict(
      armGain = 50, #250,
      gripGain = 8500,
      wristGain = 50, #250,
      )
    JoyApp.__init__(self,robot=robot,cfg=cfg,*arg,**kw)

  def onStart(self):
    sf = StickFilter(self,dt=0.05)
    sf.setIntegrator( 'joy0axis0',10 )
    sf.setIntegrator( 'joy0axis1',10 )
    sf.setIntegrator( 'joy0axis4' )
    sf.setLowpass( 'joy0axis3', 20 )
    sf.start()
    self.sf = sf
    self.ma = MoveArm(self)

  def onEvent(self,evt):
    if evt.type == JOYAXISMOTION:
      # Forward a copy of the event to the StickFilter plan
      self.sf.push(evt)
    #
    # Buttons --> press button0 to stop arm; release to start
    elif evt.type==JOYBUTTONDOWN and evt.joy==0 and evt.button==0:
      self.ma.stop()
    elif evt.type==JOYBUTTONUP and evt.joy==0 and evt.button==0:
      self.ma.start()
    #
    # Hide robot position events
    elif evt.type==CKBOTPOSITION:
      return
    JoyApp.onEvent(self,evt)

  def onStop(self):
    self.ma.onStop()

if __name__=="__main__":
  app = ArmApp()
  app.run()
