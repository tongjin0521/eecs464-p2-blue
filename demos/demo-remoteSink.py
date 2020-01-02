'''
FILE demo-RemoteSink.py
This file is used in combination with the RemoteSource demo. 
It receives and prints the commands sent by a RemoteSource object in connection with its default port
'''
from joy import JoyApp
from joy.decl import *
from joy.remote import Sink as RemoteSink

class RemoteSinkApp( JoyApp ):
  '''
  It receives and prints the data sent by RemoteSource through its port
  
  It creates a RemoteSink object which is used to receive the data in its port. It also
  creates a socket and binds its default port to it. It receives data using receive 
  attribute of the socket. It receives the messages in which are
  passed from the remote sink object to its onEvent function.
 
  This would be useful to you if you want to remotely connect a user interface to another computer.
  '''
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
    
  def onStart( self ):
    self.rs = RemoteSink(self)
    #this will start a new queue of miscellaneous events ( events without a type ) which will be trimmed to
    #  contain only the last 3.0 seconds worth of events
    self.rs.setAllowMisc(3.0)
    self.rs.start() # in case of running with no pygame
    self.showMisc = self.onceEvery(1)#This clears the miscellaneous events queue for our given frequency
  
  def onEvent( self, evt ):
    #This is used to iterate through the miscellaneous events queue for our given frequency
    if self.showMisc(): 
      if self.rs.queue:
        progress("Queue has %d messages:" % len(self.rs.queue))
        for n,msg in enumerate(self.rs.queue):
	        progress('[%0d] %s' % (n,msg))
    
    #The mouse has to be on the source window to receive inputs correctly
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
    #This will print the normal events posted from remoteSource 
    JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  print("""
    Demonstration of RemoteSink plan
    ------------------------------------
    
    Receives keyboard events from remote JoyApp(s) using RemoteSource
    
    Use in combination with the RemoteSource demo
    
    Remote events are suppressed while the mouse is over the window
  """)
  app = RemoteSinkApp()
  app.run()

