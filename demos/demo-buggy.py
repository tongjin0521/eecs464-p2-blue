from joy import *

class Buggy( Plan ):
  def __init__(self,*arg,**kw):
    Plan.__init__(self,*arg,**kw)
    self.r = self.app.robot.at
    
  def onStart( self ):
    progress("Buggy started")
  
  def onStop( self ):
    progress("Stopping")
    self.r.lwheel.set_speed(0)
    self.r.rwheel.set_speed(0)

  def behavior( self ):
    oncePer = self.app.onceEvery(0.5)
    while True:
      yield
      # Read joystick from application's StickFilter 
      sf = self.app.sf
      lspeed = sf.getValue('joy0axis1')
      rspeed = sf.getValue('joy0axis4')
      #print(lspeed,rspeed)
      
      self.r.lwheel.set_speed(lspeed*200)
      self.r.rwheel.set_speed(rspeed*200)

      if oncePer():
        progress("Buggy: left wheel %6f right wheel %6f"
          % (lspeed,rspeed))
    
class BuggyApp( JoyApp ):
  def __init__(self,robot=dict(count=2),*arg,**kw):
    cfg = dict ()
    JoyApp.__init__(self,robot=robot,cfg=cfg,*arg,**kw)
  
  def onStart(self):
    sf = StickFilter(self,dt=0.05)
    sf.setLowpass( 'joy0axis1',10)
    sf.setLowpass( 'joy0axis4',10)
    sf.start()
    self.sf = sf
    self.ma = Buggy(self) 
      
  def onEvent(self,evt):
    if evt.type == JOYAXISMOTION:
      # Forward a copy of the event to the StickFilter plan
      self.sf.push(evt)
    # Buttons --> press button0 to stop buggy; release to start
    elif evt.type==JOYBUTTONDOWN and evt.joy==0 and evt.button==0:
      self.ma.stop()
    elif evt.type==JOYBUTTONUP and evt.joy==0 and evt.button==0:
      self.ma.start()

    # Hide robot position events
    elif evt.type==CKBOTPOSITION:
      return
    JoyApp.onEvent(self,evt)

  def onStop(self):
    self.ma.onStop()
      
if __name__=="__main__":
  app = BuggyApp()
  app.run()
