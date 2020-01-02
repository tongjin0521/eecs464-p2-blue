from time import time as now
from .ckmodule import AbstractProtocol, AbstractBus, AbstractNodeAdaptor, MissingModule

# If you want to use non-default MissingModule replacements, first
#   map their NIDs to their module classes, e.g.
#   >>> import nobus as NB, dynamixel as DX
#   >>> NB.NID_CLASS[0x32] = DX.MissingDynamixel
NID_CLASS = {}

DEFAULT_PORT = None

class Protocol( AbstractProtocol ):
  """abstract superclass of all Protocol classes

  AbstractProtocol subclasses must implement the following methods:
    p.update()
    p.hintNodes(nodes)
    p.generatePNA( nid )

  AbstractProtocol instances must have the following data attributes:
    p.heartbeats -- dict -- nid to last heartbeat
  """
  def __init__(self,*args,**kw):
    AbstractProtocol.__init__(self,*args,**kw)
    self.hintNodes([])

  def update(self,t=None):
    if t is None:
        t = now()
    self._ts[0] = t

  def hintNodes( self, nodes ):
    self._ts = [now(),"No message"]
    h = {}
    for nd in nodes:
      h[nd] = self._ts
    self.heartbeats = h

  def generatePNA( self, nid ):
    return NodeAdaptor( nid )

class Bus( AbstractBus ):
  """abstract superclass of all Bus classes

  """
  def __init__(self,*args,**kw):
    AbstractBus.__init__(self,*args,**kw)

class NodeAdaptor( AbstractNodeAdaptor ):
  """abstract superclass of all ProtocolNodeAdaptor classes

  AbstractNodeAdaptor subclasses must implement the get_typecode()
  method, returning the module's type identification string
  """
  def __init__(self,nid):
    AbstractNodeAdaptor.__init__(self)
    self.nid = nid

  def get_typecode( self ):
    cls = NID_CLASS.get(self.nid,MissingModule)
    return cls.TYPECODE
