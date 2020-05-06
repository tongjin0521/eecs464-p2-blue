'''
File demo-scratchBridge.py

This file performs scratch operations with joystick and also controls
motors using scratch. This does this by exposing all properties of CKbot cluster
to scratch and using SCRATCHUPDATE to detect any change in the scratch file.
'''
from joy.decl import *
from joy import JoyApp
import re

class ScratchBridge(JoyApp):
  '''
  This class controls and performs all scratch and module operations on user input

  It passes the event to connceted scratch window to perform jotystick operations

  This class looks for SCRATCHUPDATE event if any event occurs on the scratch window and
  processes the events,
  First, it looks in the cache if it has any callbacks associated with the event.
  If not, it will try to create a new callback using the newSetter function

  It will be useful for those who need to create callbacks from strings or tuples
  '''
  def __init__(self,count=None,names={}):
    JoyApp.__init__(self,scr={},
      robot=dict(count=count,names=names))
  #This will look for a string in the "ckbot://Nx40/@set_pos" format
  REX_CLP = re.compile("\Ackbot://(\S+)\Z")

  def onStart( self ):
    self.cache = {}

  def newSetter( self, key ):
    # Pattern match with CLP pattern
    m = self.REX_CLP.match(key)
    if m is None:
      return False
    # Search for extracted CLP
    clp = m.group(1)
    try:
      fun = self.setterOf(clp)
    except (KeyError,AttributeError,ValueError) as ex:
      progress(ex)
      return False
    return fun

  def onEvent( self, evt ):
    if evt.type==TIMEREVENT:
        return
    if evt.type!=SCRATCHUPDATE:
      self.scratchifyEvent(evt)
      return JoyApp.onEvent(self,evt)
    #
    assert evt.type == SCRATCHUPDATE
    progress("Scratch sent "+evt.var+", value "+str(evt.value))
    # Lookup in cache for a setter function
    fun = self.cache.get(evt.var,None)
    if fun is None:
        progress("\t --> No handler in cache")
        assert fun is None
        fun = self.newSetter( evt.var )
        self.cache[evt.var]= fun
        if not fun:
           progress("\t '%s' was not found" % evt.var)
        else:
           progress("\t found setter :"+str(fun))
           fun(int(evt.value))
        return
    if fun is False:
        progress("\t --> known bad; ignored")
        return
    assert callable(fun)
    progress("\t --> found %r" % fun)
    fun(int(evt.value))

if __name__=="__main__":
  import sys
  print('''
  Scratch to CKBot Bridge
  -----------------------

  When running, this JoyApp exposes all properties of the CKBot
  cluster to Scratch. In addition, it emits game controller state
  updates into scratch, and mouse motions.

  Instructions:
  1. Start demo-scratchBridge.sb from command line by typing 'scrach demo-scratchBridge.sb'
  2. Run UI front end enabling mouse events: 'ctrl -e MOUSEMOTION -e MOUSEBUTTONDOWN'
  3. Run demo-scratchBridge.py and give it the number of modules you have connected; in this documentation we will assume there are two: Nx09 and Nx59
  4. To move the cat, click on the green flag in the scratch window. If you now move the mouse over the JoyApp UI window, the cat will make corresponding motions
  5. To make the cat talk, left-click your mouse
  6. You can also control motors from scratch. To move the motors, create a variable corresponding to your module name, with the following schema: if your module is Nx40, use ckbot://Nx40/@set_pos . Then click on Sprite 2 (the "Move Me!" sprite) and update the variable in the script which says      'when Sprite2 clicked'. Now activate this sprite by clicking on it, and drag it up and down to control the motor.
  7. If you hit the 'space' key in scratch, it will wiggle the motors three times. You will need to set variables accordingly.

  Many different event types are emitted to scratch, and some events (like mouse events) produce multiple sensor updates. Search for SCRATCHY in joy/__init__.py for a complete list of event types that are converted.
  ''')
  if len(sys.argv)!=2:
    raise ValueError("Must give number of modules on commandline")
  sb = ScratchBridge(count=int(sys.argv[1]))
  DEBUG = 'S'
  sb.run()
