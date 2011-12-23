from joy import *
import re

class ScratchBridge(JoyApp):
  def __init__(self,count=None,names={},walk=False):
    JoyApp.__init__(self,scr={},
      robot=dict(count=count,names=names,walk=walk))

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
    try:
      fun = self.robot.setterOf(clp)
    except (KeyError,AttributeError,ValueError), ex:
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
      # Lookup in cache 
      fun = self.cache.get(evt.var,None)
      # If cache hit --> call setter function
      if fun: 
        fun(int(evt.value))
      elif fun is False: # cached as a bad name --> ignore
        pass
      else: # New --> lookup in robot, and cache result
        assert fun is None
        fun = self.newSetter( evt.var )
        if fun:
          fun(int(evt.value))
    elif evt.type != TIMEREVENT:
      self.scratchifyEvent(evt)
      JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  import sys
  print '''
  Scratch to CKBot Bridge
  -----------------------
  
  When running, this JoyApp exposes all properties of the CKBot
  cluster to Scratch. In addition, it emits game controller state
  updates into scratch.
  
  The commandline version (which you are running now) expects
  the number of clusters to be given as a parameter, and will
  walk all those clusters, but the ScratchBridge class does not
  require this. If the number of clusters is not specified,
  the default Cluster.populate() settings are used.
  '''
  if len(sys.argv)==2:
    sb = ScratchBridge(count=int(sys.argv[1]),walk=True)
  else:
    sb = ScratchBridge(walk=True)
    
  DEBUG = 'S'
  sb.run()
  
