'''
FILE demo-remoteSource.py

This file is used in combination with the RemoteSink demo.
It sends commands to RemoteSink using the specified port and hostname specified  by the user
'''
from sys import argv
from joy import JoyApp
from joy.decl import *
from joy.pygix import SCHD
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
    if SCHD != "pygame":
        raise RuntimeError("\n"+"*"*40+"\nRemote source should not be used without activating the pygame scheduler. Try something like: 'PYGIXSCHD=pygame python %s' on your commandline" % argv[0])
    self.dst = kw.get('sink',RemoteSource.DEFAULT_SINK)
    if 'sink' in kw:
      del kw['sink']
    self.evts = kw.get('evts',{KEYDOWN,KEYUP,MIDIEVENT,JOYAXISMOTION,JOYBUTTONDOWN})
    if 'evts' in kw:
      del kw['evts']
    if 'cfg' not in kw:
        kw.update(cfg={})
    kw['cfg'].update(remote=None)
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
    if evt.type in self.evts:
      # For pressing space --> send a custom message instead
      if evt.type == KEYDOWN and evt.key == K_SPACE:
        self.rs.sendMsg( note = "This is a custom message" )
      else: # else --> send the event
        self.rs.push( evt )
      # For KEYDOWN --> process locally, AND send over
      if evt.type == KEYDOWN:
        if evt.key == K_q:
          progress("Use 'q' to terminate remote; ESC to terminate local AND remote")
        else:
          JoyApp.onEvent(self,evt)
      return
    #mousemotion events are ignored here
    elif evt.type == MOUSEMOTION:
      return
    JoyApp.onEvent(self,evt)

if __name__=="__main__":
  from argparse import ArgumentParser
  p = ArgumentParser(description="""
    Demonstration of Controllers plan
    ------------------------------------

    Sends keyboard events to a remote JoyApp, specified on the commandline as two parameters: host port
    If the port is not specified, default port is used

  """)
  from joy import decl
  evnm = {nm : getattr(decl,nm) for nm in [
    'MIDIEVENT', 'MOUSEBUTTONDOWN', 'MOUSEBUTTONUP', 'MOUSEMOTION',
    'JOYAXISMOTION', 'JOYBALLMOTION', 'JOYBUTTONDOWN', 'JOYBUTTONUP', 'JOYHATMOTION', 'KEYDOWN', 'KEYUP', 'HAT_CENTERED', 'HAT_DOWN', 'HAT_LEFT', 'HAT_LEFTDOWN', 'HAT_LEFTUP', 'HAT_RIGHT', 'HAT_RIGHTDOWN', 'HAT_RIGHTUP', 'HAT_UP', 'CKBOTPOSITION', 'ACTIVEEVENT'
  ]}
  p.add_argument('--events','-e',action='append', help='Include event type '+repr(evnm.keys()))
  p.add_argument('--dst','-d',action='store',default=RemoteSource.DEFAULT_SINK[0],help='Address of event receiver')
  p.add_argument('--dport','-p',action='store',default=str(RemoteSource.DEFAULT_SINK[1]),help='Port of event receiver')
  from sys import argv
  args = p.parse_args(argv[1:])

  evts = set()
  if args.events is not None:
      evts = { evnm[nm.upper()] for nm in args.events }
  else:
      evts = {KEYDOWN,KEYUP,JOYAXISMOTION,JOYBUTTONDOWN}
  progress("*** Events "+repr(evts) )
  sink = (args.dst,int(args.dport))
  progress("*** Destination "+repr(sink) )
  app = RemoteSourceApp(sink=sink,evts=evts)
  app.run()
