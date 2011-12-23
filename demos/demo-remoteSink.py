from joy import *
from joy.remote import Sink as RemoteSink

class RemoteSinkApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
    
  def onStart( self ):
    self.rs = RemoteSink(self)
  
  def onEvent( self, evt ):
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

