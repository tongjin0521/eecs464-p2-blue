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
  import sys
  if len(sys.argv)>2:
    app = RemoteSourceApp(sink=(argv[1],int(argv[2])))
  else:
    app = RemoteSourceApp()
  app.run()

