"""
  The ckbots.logical module provides classes to create representations
  of modules inside a cluster. It directly includes classes representing
  modules on a CAN-bus (for historical reasons), and imports additional
  module classes from pololu and hitec. The top of the classes heirarchy
  for modules and their NodeAdaptor-s are found in ckmodule.

  Main uses of this module:
  (*) query the object dictionary of a module by its logical name
  (*) send a position command using a process message

  The top level of this module is cluster. Typically users will create a
  cluster  to represent a set of modules that can communicate on the same
  CANBus. Modules can be  addressed through logical names via the Attributes
  class.
"""
from re import compile as re_compile
from time import sleep, time as now
from traceback import extract_stack

from .ckmodule import ( 
    AbstractProtocol, progress, MissingModule, Module, DebugModule, PermissionError
)
from . import polowixel
from . import pololu
from . import dynamixel
from .defaults import DEFAULT_ARCH, DEFAULT_PORT

def nids2str( nids ):
  return ",".join(["Nx%02x" % nid for nid in nids])

class ModulesByName(object):
  """
  Concrete class with a cluster's attributes.

  The cluster dynamically adds named attributes to instances of this class to
  provide convenient names for modules
  """
  def __init__(self):
    self.__names = set()

  def _add( self, name, value ):
    setattr( self, name, value )
    self.__names.add(name)

  def _remove( self, name ):
    delattr( self, name )
    self.__names.remove(name)

  def __iter__(self):
    plan = list(self.__names)
    plan.sort()
    return iter(plan)

class DiscoveryError( Exception ):
  """
  Exception class for discovery failures
  """
  def __init__(self,msg,**kw):
    """
    ATTRIBUTES:
        timeout -- number -- seconds to timeout
        required -- set -- required node ID-s
        found -- set -- node ID-s found
        count -- number -- node count required
    """
    Exception.__init__(self,msg)
    self.timeout = kw.get('timeout',0)
    self.count = kw.get('count',0)
    self.required = kw.get('required',set([]))
    self.found = kw.get('found',set([]))

class DelayedPermissionError( PermissionError ):
  """
  Callable object returned when setters or getters with
  are obtained for cluster properties that cannot be (resp.)
  written or read.

  A DelayedPermissionError stores the stack at its initialization
  to make it easier for the subsequent error to be traced back to
  its original source.
]
  If called, a DelayedPermissionError raises itself.
  """
  def __init__(self, *arg, **kw):
    PermissionError.__init__(self,*arg,**kw)
    self.init_stack = extract_stack()

  def __call__( self, *arg, **kw ):
    raise self

class Cluster(dict):
  """
  Concrete class representing a CKBot cluster, which is a collection of
  modules residing on the same bus.

  A Cluster contains a Protocol class to manage communication with the
  modules.

  A Cluster instance is itself a dictionary of modules, addressed by
  their node ID-s. This dictionary is populated by the .populate()
  method. Clusters also implement the reflection iterators itermodules, and iterprop.

  Typically, users will use the convenience attribute .at, which
  provides syntactic sugar for naming modules in a cluster using names
  defined when the cluster is .populate()-ed. These allow ipython tab
  completion to be used to quickly explore which modules are available.

  Typical use:
    >>> c = Cluster()
    >>> c.populate(3,{ 0x91 : 'left', 0xb2 : 'head',0x5d : 'right'} )
    >>> for m in c.itervalues():
    >>>   m.get_od(c.p)
    >>> c.at.head.set_pos(4500) # via process message
  """
  def __init__(self,arch=None,port=None,*args,**kwargs):
    """
    Create a new cluster. Optionally, also .populate() it

    INPUT:
      arch -- optional -- python module containing arch.Bus and
        arch.Protocol to use for low level communication. Defaults
        to DEFAULT_ARCH

        Can also be a Protocol instance ready to be used.

        Supported architectures include:
          dynamixel -- Robotis Dynamixel RX, EX and MX
          pololu -- pololu Maestro servo controllers
          nobus -- software simulated modules for no-hardware-needed
            testing of code.
          polowixel -- pololu Maestro via Wixel wireless, with wixel position feedpack

      port -- specification of the communication port to use, as per
        ckbot.port2port.newConnection. port defaults to DEFAULT_PORT, and
        is ignored if arch is an initialized AbstractProtocol instance.
        If DEFAULT_PORT is None (default value), looks for DEFAULT_PORT in arch

        This can be used to specify serial devices and baudrates, e.g.
        port = dict(TYPE='tty',glob="/dev/ttyACM1",baudrate=115200)

      *argc, **kw -- if any additional parameters are given, the
        .populate(*argc,**kw) method is invoked after initialization

    ATTRIBUTES:
      p -- instance of Protocol for communication with modules
      at -- instance of the Attributes class.
      limit -- float -- heartbeat time limit before considering node dead
      _updQ -- list -- collection of objects that need update() calls
    """
    dict.__init__(self)
    if arch is None:
      arch = DEFAULT_ARCH
    if isinstance(arch,AbstractProtocol):
      self.p = arch
    else:
      if port is None:
        port = DEFAULT_PORT
      if port is None:
        port = arch.DEFAULT_PORT
      self.p = arch.Protocol(bus = arch.Bus(port=port))
    assert isinstance(self.p,AbstractProtocol)
    self._updQ = [self.p]
    self.at = ModulesByName()
    self.limit = 2.0
    if args or kwargs:
      return self.populate(*args,**kwargs)

  def populate(self, count = None, names = {}, timeout=2, timestep=0.1,
                required = set(), fillMissing=None,
                autonamer=lambda nid : "Nx%02X" % nid ):
    """
    Tries to populate the cluster based on heartbeats observed on the bus.
    Will terminate when either at least count modules were found or timeout
    seconds have elapsed. While waiting, checks bus once every timestep seconds.
    If timed out, raises an IOError exception.

    If the bus already got all the heartbeats needed, populate() should
    terminate without sleeping.

    If provided, names gives a dictionary of module names based on their node
    ID.	Node IDs that aren't found in the dictionary have a name automatically
    generated from the ID by calling autonamer.

    INPUT:
      count, timeout, timestep, required, fillMissing-- see self.discover()
      fillMissing -- class / bool -- fills in any missing yet required modules
            with instances of this class. If boolean true, uses MissingModule
            NOTE: the nobus mechanism bypasses this; use nobus.NID_CLASS
      names -- dictionary of Modules names based with node id as their key.
      autonamer -- names the modules if no names are given.
    """
    self.clear()
    if fillMissing is None:
      exc = DiscoveryError
    else:
      exc = None
    required = set(required)
    if required and (count is None or count<len(required)):
       count = len(required)
    if names != {} and count is None:
       count = len(names)
    nids = self.discover(count,timeout,timestep,required,raiseClass = exc)
    for nid in nids:
      name = names.get(nid, autonamer(nid))
      mod = self.newModuleIX( nid, name )
      self.add(mod)
    if fillMissing:
      if type(fillMissing) is not type:
        fillMissing = MissingModule
      for nid in required - nids:
        name = names.get(nid, autonamer(nid))
        mod = fillMissing( nid, name )
        self.add(mod)

  def discover( self, count = 0, timeout=2, timestep=0.1, required=set(), raiseClass=DiscoveryError ):
    """
    Discover which nodes are in the cluster.
    Termination condition for discovery is that at least count modules were
    discovered, and that all of the required modules were found.
    If this hasn't happened after a duration of timeout seconds (+/- a timestep),
    discover raises a DiscoveryError.
    In the special case of count==0 and required=set(), discover collects
    all node ID-s found until timeout, and does not return an error.
    INPUT:
      count -- number of modules -- if 0 then collects until timeout
      timeout -- time to listen in seconds
      timestep -- how often to read the CAN buffer
      required -- set -- requires that all these nids are found
      raiseClass -- class -- exception class to raise on timeout
    OUTPUT:
      python set of node ID numbers
    """
    required = set(required)
    # If any number of nodes is acceptable --> wait and collect them
    if not count and not required:
      progress("Discover: waiting for %g seconds..." % timeout)
      sleep(timeout)
      nids = self.getLive(timeout)
      progress("Discover: done. Found %s" % nids2str(nids))
      return nids
    # If some nodes are required --> hint them to protocol
    if required:
      self.p.hintNodes( required )
    # else --> collect nodes with count limit, timeout, required
    time_end = now()+timeout
    nids = self.getLive(timeout)
    while (len(nids) < count) or not (nids >= required):
      sleep(timestep)
      nids = self.getLive(timeout)
      progress("Discover: found %s" % nids2str(nids))
      if time_end < now():
        if raiseClass is None:
          break
        raise raiseClass("Module discovery timeout",
          timeout=timeout, found=nids, required=required, count=count )
    return nids

  def update(self,t=None):
    """Allow stateful members to update; propagates to sub-objects"""
    if t is None:
        t = now()
    for m in self._updQ:
      m.update(t)

  def off( self ):
    """Make all servo or motor modules go slack"""
    for m in self.itermodules():
      if hasattr(m,'go_slack') and callable(getattr(m,'go_slack')):
        m.go_slack()

  def who( self, t = 10 ):
    """
    Show which modules are currently visible on the bus
    runs for t seconds
    """
    t0 = now()
    while now()-t0<t:
      lst = []
      for nid in self.getLive():
        if nid in self:
          lst.append( '%02X:%s' % (nid,self[nid].name) )
        else:
          lst.append( '%02X:<?>' % nid )
      print("%.2g:" % (now()-t0),", ".join(lst))
      sleep(1)

  def add( self, *modules ):
    """
    Add the specified modules (as returned from .newModuleIX())
    """
    for mod in modules:
      progress("Adding %s %s" % (mod.__class__.__name__,mod.name) )
      assert isinstance(mod,Module)
      self.at._add(mod.name, mod)
      if hasattr(mod,"update") and callable(mod.update):
        self._updQ.append(mod)
      self[mod.node_id] = mod
    return self

  def newModuleIX(self,nid,name=None):
    """
    Build the interface for a module
    INPUTS
        nid -- int -- node identifier for use on the bus
        name -- string -- name for the module, or None if regenerating
            an existing module whose name is already known
    OUTPUTS
        mod -- Module subclass representing this node
    """
    nid = int(nid)
    pna = self.p.generatePNA(nid)
    if name is None:
      name = self[nid].name
    tc =  pna.get_typecode()
    mod = Module.newFromDiscovery(nid, tc, pna)
    mod.name = name
    return mod

  def __delitem__( self, nid ):
    self.at._remove( self[nid].name )
    dict.__delitem__(self,nid)

  def itermodules( self ):
    for mnm in self.at:
      yield getattr(self.at,mnm)

  def iterprop( self, attr=False, perm='' ):
    for mod in self.itermodules():
      if attr:
        for prop in mod.iterattr(perm):
          yield mod.name + "/@" + prop
      for prop in mod.iterprop(perm):
        yield mod.name + "/" + prop

  def getLive( self, limit=None ):
    """
    Use heartbeats to get set of live node ID-s
    INPUT:
      limit -- float -- heartbeats older than limit seconds in
         the past are ignored
    OUTPUT:
      python set of node ID numbers
    """
    if limit is None:
      limit = self.limit
    t0 = now()
    self.p.update(t0)
    s = set( ( nid
      for nid,(ts,_) in self.p.heartbeats.items()
      if ts + limit > t0 ) )
    return s

  REX_PROP = re_compile("([a-zA-Z_]\w*)(/)((?:[a-zA-Z_]\w*)|(?:0x[a-fA-F0-9]{4}))")
  REX_ATTR = re_compile("([a-zA-Z_]\w*)(/@)([a-zA-Z_]\w*)")

  @classmethod
  def parseClp( cls, clp ):
    """
    parse a class property name into head and tail parts

    use this method to validate class property name syntax.

    OUTPUT: kind, head, tail
        where kind is one of "/", "/@"
    """
    m = cls.REX_PROP.match(clp)
    if not m: m = cls.REX_ATTR.match(clp)
    if not m:
      raise ValueError("'%s' is not a valid cluster property name" % clp )
    return m.group(2),m.group(1),m.group(3)

  def modOfClp(self, clp ):
    """
    Find the module containing a given property
    """
    kind,head,tail = self.parseClp(clp)
    # Property or Attribute
    if kind[:1]=="/":
      return getattr( self.at, head )

  def _getAttrOfClp( self, clp, attr ):
    """(private)
    Obtain python attribute of a cluster property identified by a clp
    """
    kind,head,tail = self.parseClp(clp)
    # Property or Attribute
    if kind[:1]=="/":
      try:
        mod = getattr( self.at, head )
        return mod._getModAttrOfClp( kind+tail, attr )
      except AttributeError:
        raise KeyError("'%s' is not the name of a module in .at" % head )

  def getterOf( self, clp ):
    """
    Obtain a getter function for a cluster property

    If property is not readable, returns a DelayedPermissionError
    """
    if self._getAttrOfClp(clp,'isReadable')():
      return self._getAttrOfClp(clp,'get_sync')
    return DelayedPermissionError("Property '%s' is not readable" % clp)

  def setterOf( self, clp ):
    """
    Obtain a setter function for a cluster property

    If property is not writeable, returns a DelayedPermissionError
    """
    if self._getAttrOfClp(clp,'isWritable')():
      return self._getAttrOfClp(clp,'set')
    return DelayedPermissionError("Property '%s' is not writable" % clp)

Module.Types.update(
    # Inherit all module ID strings defined in dynamixel.MODELS
    { tc : mc for tc,(mm,mc) in dynamixel.MODELS.items() },
    PolServoModule = pololu.ServoModule,
    PoloWixelCR = polowixel.ServoModuleCR,
    PoloWixelUB = polowixel.ServoModuleUB,
)
Module.Types[MissingModule.TYPECODE] = MissingModule
Module.Types[DebugModule.TYPECODE] = DebugModule
