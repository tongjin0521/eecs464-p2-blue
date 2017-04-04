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
    if evt.type in set([KEYDOWN,KEYUP,JOYAXISMOTION]):
      # Process keydown events so that 'q' key will work
      if evt.type == KEYDOWN:
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
  from joy.remote import DEFAULT_PORT
  if len(sys.argv)>1:
    if len(sys.argv)>2:
      port = int(sys.argv[2])
    else:
      port = DEFAULT_PORT
      progress("**** Using default port "+str(DEFAULT_PORT))
    app = RemoteSourceApp(sink=(sys.argv[1],port))
  else:
    app = RemoteSourceApp()
  app.run()

