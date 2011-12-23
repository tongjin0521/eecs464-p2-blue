from joy import *

class JoyToScr(JoyApp):
  def __init__(self):
    JoyApp.__init__(self,scr={})
    self.speed = 10
  
  def onStart( self ):
    self.scr.sensorUpdate( j0ax1=0, j0ax0=0 )
    self.scr.broadcast( 'j0btn0', 'j0btn1' )
    
  def onEvent( self, evt ):
    if evt.type==JOYAXISMOTION and evt.joy==0:
      if evt.axis==0:
        self.scr.sensorUpdate( j0ax0=int(evt.value*self.speed) )
      elif evt.axis==1:
        self.scr.sensorUpdate( j0ax1=-int(evt.value*self.speed) )
    elif evt.type==JOYBUTTONDOWN and evt.joy==0:
      if evt.button==0:
        self.scr.broadcast( 'j0btn0' )
      elif evt.button==1:
        self.scr.broadcast( 'j0btn1' )
    elif evt.type != TIMEREVENT:
      JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  print '''
  DEMO: connecting joystick to Scratch
  ------------------------------------
  
  When this program runs, axes 0 and 1 of joystick 0 are mapped to
  Scratch sensors j0ax0 and j0ax1; joystick buttons 0 and 1 are
  mapped to events j0btn0 and j0btn1
  
  '''
  j2s = JoyToScr()
  j2s.run()
  
