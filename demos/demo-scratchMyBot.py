from joy import *

class ScrMyBot(JoyApp):
  def __init__(self):
    JoyApp.__init__(self,scr={},robot=dict(count=1))
    self.gain = 100
    self.limit = 5000
  
  def onStart( self ):
    self.aNode = self.robot.values()[0]
    
  def onEvent( self, evt ):
    if evt.type==SCRATCHUPDATE and evt.scr==0:
      if evt.var=='robotPos1':
        pos = evt.value * self.gain
        if pos>self.limit: pos = self.limit
        elif pos<-self.limit: pos = -self.limit
        self.aNode.set_pos( pos )
    elif evt.type != TIMEREVENT:
      JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  print '''
  DEMO: connecting Scratch sprite to robot
  ----------------------------------------
  
  When this program runs, the x position of the cat sprite on 
  the screen controls a servo in the robot.
  
  '''
  smb = ScrMyBot()
  smb.run()
  
