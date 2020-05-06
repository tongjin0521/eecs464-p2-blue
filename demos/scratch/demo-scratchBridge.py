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
      self.cache[key] = None
      return None
    # Search for extracted CLP
    clp = m.group(1)
    print("ClP="+clp)
    try:
      fun = self.setterOf(clp)
    except (KeyError,AttributeError,ValueError) as ex:
      fun = ex
    # If failed --> invalid property name
    if isinstance(fun,StandardError):
      self.cache[key] = False
      return False
    # Record new cache value
    self.cache[key]= fun
    return fun

  def onEvent( self, evt ):
    if evt.type==SCRATCHUPDATE:
      # Lookup in cache for a setter function
      fun = self.cache.get(evt.var,None)
      print(evt.var+" Event Value "+str(evt.value))
      print("Function before if:"+str(fun))
      # If cache hit --> call setter function
      if fun:
        fun(int(evt.value))
        print("function")
      elif fun is False: # cached as a bad name --> ignore
        print("None")
        pass
      else: # New --> lookup in robot, and cache result
        assert fun is None
        fun = self.newSetter( evt.var )
        print("Function in else:"+str(fun))
        print("New function")
        if fun:
          print("New function created")
          fun(int(evt.value))
    elif evt.type != TIMEREVENT:
      self.scratchifyEvent(evt)
      JoyApp.onEvent(self,evt)

if __name__=="__main__":
  import sys
  print('''
  Scratch to CKBot Bridge
  -----------------------

  When running, this JoyApp exposes all properties of the CKBot
  cluster to Scratch. In addition, it emits game controller state
  updates into scratch.

  Instructions:
  1. Start demo-scratchBridge.sb from command line by typing 'scrach demo-scratchBridge.sb'
  2. Run demo-scratchBridge.py in another terminal
  3. To move the cat, click on the green flag in the scratch window and move the controller's left
     analog
  4. To make the cat talk, go to the pygame window and click the controller's face buttons
  5. To move the motors, create a variable ckbot://Nx40/@set_pos corresponding to
     your module name. Then click on Sprite 2 and update the variable in the script which says
     'when Sprite2 clicked'
  5. Click and move the Nx36 icon to move the motors

  The commandline version (which you are running now) expects
  the number of modules to be given as a parameter, but the
  ScratchBridge class does not require this. If the number of
  modules is not specified, the default Cluster.populate() settings are used.
  ''')
  if len(sys.argv)==2:
    sb = ScratchBridge(count=int(sys.argv[1]))
  else:
    sb = ScratchBridge()

  DEBUG = 'S'
  sb.run()
