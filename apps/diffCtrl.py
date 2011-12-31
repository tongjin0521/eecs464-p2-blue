# Just a copy of the remoteSource demo with fixed host and port
from joy import *
from joy.remote import Source as RemoteSource

class RemoteSourceApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)


  def onStart( self ):
    self.rs = RemoteSource(self, dst=('172.16.16.135', 31313))
    self.rs.start()

  def onEvent( self, evt ): 
    if evt.type in set([KEYDOWN, KEYUP]):
      print evt
      self.rs.push( evt )
      return   
    JoyApp.onEvent( self, evt )
   

if __name__=="__main__":
  print """
    Demonstration of RemoteSource plan
    ------------------------------------
    
    Sends keyboard events to a remote JoyApp, specified on the commandline as two parameters: host port
    
  """
  app = RemoteSourceApp()
  app.run()

