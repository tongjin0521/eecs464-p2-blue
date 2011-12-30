# Just a copy of the remoteSource demo with fixed host and port
from joy import *
from joy.remote import Source as RemoteSource

class RemoteSourceApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
    self.dst = kw.get('sink',RemoteSource.DEFAULT_SINK)
    
  def onStart( self ):
    self.rs = RemoteSource(self)
    self.rs.start()
    
  def onEvent( self, evt ):
    if evt.type in set([KEYDOWN, KEYUP]):
      self.rs.push( evt )
      return
    elif evt.type == MOUSEMOTION:
      return
    JoyApp.onEvent(self,evt)

if __name__=="__main__":
  print """
    Demonstration of RemoteSource plan
    ------------------------------------
    
    Sends keyboard events to a remote JoyApp, specified on the commandline as two parameters: host port
    
  """
  app = RemoteSourceApp(sink=('172.16.16.135', 31313))
  app.run()

