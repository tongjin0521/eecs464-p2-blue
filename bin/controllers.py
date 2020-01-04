#!/usr/bin/env python
# NOTE: this program should always run under python2
#   at least until MIDI starts working in pygame on python 3.x
'''
FILE controllers.py

Use pygame to generate controller event streams over UDP.

In pyckbot 1.x this was "demo-remoteSource", but starting with pyckbot 3.0 this
is the main way in which controller and keyboard events are produced, and
all JoyApp-s run (by default) without pygame.

To force use of pygame, use PYGIXSCHD=pygame in the unix shell environment
'''
from sys import argv
from joy import JoyApp
from joy.decl import *
from joy.remote import Source as RemoteSource
from joy.events import event_name
from joy.misc import requiresPyGame
requiresPyGame()

class RemoteSourceApp( JoyApp ):
  '''
  This class is used to send events to remote.Sink from the host to the port specified by the user

  It creates a remote.Source object and passes the destination port specified by thes user to the
  remote.Source. This remote.Source binds the hostname to a socket to send messages to the specified
  port

  By default, will only forward KEYDOWN events
  '''
  def __init__(self,*arg,**kw):
    self.dst = kw.get('sink',RemoteSource.DEFAULT_SINK)
    if 'sink' in kw:
      del kw['sink']
    self.evts = kw.get('evts',{KEYDOWN})
    if 'evts' in kw:
      del kw['evts']
    if 'cfg' not in kw:
        kw.update(cfg={})
    kw['cfg'].update(remote=None,midi=True)
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
      if evt.type == KEYDOWN and evt.key == K_RSHIFT:
        progress("The RSHIFT key sends a custom message (as demo)")
        self.rs.sendMsg( note = "This is a custom message" )
      else: # else --> send the event
        self.rs.push( evt )
      # For KEYDOWN --> process locally, AND send over
      if evt.type == KEYDOWN:
        if evt.key == K_q:
          progress("(say) Use 'q' key to terminate remote; 'escape' key to terminate local AND remote")
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
    Controllers interface for pyckbot
    ------------------------------------

    Events to a remote JoyApp, specified on the commandline as two parameters:
    (UDP) port and destination IP. These default to loopback and the default
    port for remoteSink plans.

    Starting with pyckbot 3.0, this program is the primary way to control JoyApp
    applications, and is the only program in the core codebase to use pygame.

  """)
  from joy import decl
  evnm = {nm : getattr(decl,nm) for nm in [
    'MIDIEVENT', 'MOUSEBUTTONDOWN', 'MOUSEBUTTONUP', 'MOUSEMOTION',
    'JOYAXISMOTION', 'JOYBALLMOTION', 'JOYBUTTONDOWN', 'JOYBUTTONUP', 'JOYHATMOTION', 'KEYDOWN', 'KEYUP', 'HAT_CENTERED', 'HAT_DOWN', 'HAT_LEFT', 'HAT_LEFTDOWN', 'HAT_LEFTUP', 'HAT_RIGHT', 'HAT_RIGHTDOWN', 'HAT_RIGHTUP', 'HAT_UP', 'CKBOTPOSITION', 'ACTIVEEVENT'
  ]}
  p.add_argument('--events','-e',action='append', help='Include event type '+repr(evnm.keys()))
  p.add_argument('--block','-b',action='append', help='Block event type '+repr(evnm.keys()))
  p.add_argument('--dst','-d',action='store',default=RemoteSource.DEFAULT_SINK[0],help='Address of event receiver')
  p.add_argument('--dport','-p',action='store',default=str(RemoteSource.DEFAULT_SINK[1]),help='Port of event receiver')
  p.add_argument('--nodefault','-n',action='store_true',help='include no default event types')
  from sys import argv
  args = p.parse_args(argv[1:])

  evts = {} if args.nodefault else {
    KEYDOWN,KEYUP,MIDIEVENT,JOYAXISMOTION,JOYBUTTONDOWN
  }
  if args.events is not None:
      evts = { evnm[nm.upper()] for nm in args.events }
  if args.block is not None:
      evts = evts - { evnm[nm.upper()] for nm in args.block }
  progress("*** Events "+repr([event_name(eid) for eid in evts]) )
  sink = (args.dst,int(args.dport))
  progress("*** Destination "+repr(sink) )
  app = RemoteSourceApp(sink=sink,evts=evts)
  app.run()
