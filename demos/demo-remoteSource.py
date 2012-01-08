from joy import *
from joy.remote import Source as RemoteSource

class RemoteSourceApp( JoyApp ):
  def __init__(self,*arg,**kw):
    self.dst = kw.get('sink',RemoteSource.DEFAULT_SINK)
    if kw.has_key('sink'):
      del kw['sink']
    JoyApp.__init__(self,*arg,**kw)
    
  def onStart( self ):
    self.rs = RemoteSource(self, self.dst)
    self.rs.start()
    
  def onEvent( self, evt ):
    if evt.type in set([KEYDOWN, KEYUP]):
      print ">>>>",
      JoyApp.onEvent(self,evt)
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
    app = RemoteSourceApp(sink=(sys.argv[1],int(sys.argv[2])))
  else:
    app = RemoteSourceApp()
  app.run()

