from joy import JoyApp
from joy.decl import *
from joy.remote import Sink as RemoteSink

class RemoteSinkApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
    
  def onStart( self ):
    self.rs = RemoteSink(self)
    self.rs.setAllowMisc(True)
    self.rs.start() # in case of running with no pygame
    self.showMisc = self.onceEvery(1)
  
  def onEvent( self, evt ):
    if self.showMisc():
      if self.rs.queue:
        progress("Queue has %d messages:" % len(self.rs.queue))
        for n,msg in enumerate(self.rs.queue):
	  progress('[%0d] %s' % (n,msg))
    if evt.type == ACTIVEEVENT:
      if evt.gain==1 and self.rs.isRunning():
        progress("(say) ignoring remote events")
        self.rs.stop()
        return
      elif evt.gain==0 and not self.rs.isRunning():
        progress("(say) allowing remote events")
        self.rs.start()
        return
    elif evt.type==MOUSEMOTION:
      # suppressed for output readability
      return 
    JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  print """
    Demonstration of RemoteSink plan
    ------------------------------------
    
    Receives keyboard events from remote JoyApp(s) using RemoteSource
    
    Use in combination with the RemoteSource demo
    
    Remote events are suppressed while the mouse is over the window
  """
  app = RemoteSinkApp()
  app.run()

