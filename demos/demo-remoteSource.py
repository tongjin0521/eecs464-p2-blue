'''
FILE demo-remoteSource.py

This file is used in combination with the RemoteSink demo. 
It sends commands to RemoteSink using the specified port and hostname specified  by the user
'''
from joy import JoyApp
from joy.decl import *
from joy.remote import Source as RemoteSource

class RemoteSourceApp( JoyApp ):
  '''
  This class is used to send events to remoteSink from the host to the port specified by the user
  
  It creates a remoteSource object and passes the destination port specified by thes user to the 
  remoteSource. This remotesource binds the hostname to a socket to send messages to the specified 
  port

  This would be useful to you if you want to remotely connect a user interface to another computer.
  '''
  def __init__(self,*arg,**kw):
    #it extracts the sink destination port from the arguement and deletes it thereafter
    self.dst = kw.get('sink',RemoteSource.DEFAULT_SINK)
    if kw.has_key('sink'):
      del kw['sink']
    JoyApp.__init__(self,*arg,**kw)
    
  def onStart( self ):
    #this passes the extracted port as an arguement to a RemoteSource object
    self.rs = RemoteSource(self, self.dst)
    self.rs.start()
    
  def onEvent( self, evt ):
    '''
    it pushes the below events to the remotesource object and this events will be sent to remoteSink
    which will print the events
    ''' 
    if evt.type in set([KEYDOWN,KEYUP,JOYAXISMOTION,JOYBUTTONDOWN]):
      # Process keydown events 
      if evt.type == KEYDOWN:
        JoyApp.onEvent(self,evt)
      if evt.type == KEYDOWN and evt.key == K_SPACE:
        self.rs.sendMsg( note = "This is a custom message" )
      else:
        self.rs.push( evt )
      return
    #mousemotion events are ignored here
    elif evt.type == MOUSEMOTION:
      return
    JoyApp.onEvent(self,evt)

if __name__=="__main__":
  print """
    Demonstration of RemoteSource plan
    ------------------------------------
    
    Sends keyboard events to a remote JoyApp, specified on the commandline as two parameters: host port
    If the port is not specified, default port is used
    
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

