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
import re
from time import sleep, time as now
from warnings import warn
from traceback import extract_stack

try:
  import can
except ImportError:
  warn("CAN support could not be imported")
  can = None

import P18F2680
from ckmodule import *

import pololu
import hitec
import dynamixel

DEFAULT_BUS = dynamixel

def nids2str( nids ):
  return ",".join(["Nx%02x" % nid for nid in nids])

def crop( val, lower, upper ):
  return max(min(val,upper),lower)
  
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

class DiscoveryError( StandardError ):
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
    StandardError.__init__(self,msg)
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
  Concrete class representing a CKBot cluster.

  A Cluster instance is a dictionary of modules. A Cluster also contains a 
  Protocol class to communicate on the CANBus of the cluster, and the
  convenience attribute at, which provides syntactic sugar for naming modules

  Typical use:
    >>> c = Cluster()
    >>> c.populate(3,{ 0x91 : 'left', 0xb2 : 'head',0x5d : 'right'} )
    >>> for m in c.itervalues():
    >>>   m.get_od(c.p)
    >>> c.at.head.od.set_pos( 3000 ) # via object dictionary
    >>> c.at.head.set_pos(4500) # via process message
  """
  def __init__(self,protocol=None,*args,**kwargs):
    """
    Concrete constructor.
      bus -- Bus class or bus instance to use; default is DEFAULT_BUS
      
    ATTRIBUTES:
      p -- instance of Protocol for communication with modules
      at -- instance of the Attributes class.
      limit -- float -- heartbeat time limit before considering node dead
    """
    dict.__init__(self,*args,**kwargs)
    if protocol is None:
      self.p = DEFAULT_BUS.Protocol()
    elif type(protocol)==type:
      self.p = protocol()
    else:
      self.p = protocol
    self.at = ModulesByName()
    self.limit = 2.0

  def populate(self, count = None, names = {}, timeout=2, timestep=0.1,
                required = set(), fillMissing=None, walk=False,
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
      names -- dictionary of Modules names based with node id as their key.
      walk -- bool -- if true, walks each module to indentify its interface
      autonamer -- names the modules if no names are given.
    """
    self.clear()
    if fillMissing is None:
      exc = DiscoveryError
    else:
      exc = None
    required = set(required)
    nids = self.discover(count,timeout,timestep,required,raiseClass = exc)
    for nid in nids:
      name = names.get(nid, autonamer(nid))
      mod = self.newModuleIX( nid, name )
      self.add(mod)
      if walk:
        mod.get_od()
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
    elif required and (count is 0 or len(required)==count):
      # If we only want the required nodes, hint them to protocol
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
        raise raiseClass("CAN discovery timeout",
          timeout=timeout, found=nids, required=required, count=count )
    return nids

  def off( self ):
    """Make all servo or motor modules currently live go slack"""
    for nid in self.getLive():
      m = self[nid]
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
        if self.has_key(nid):
          lst.append( '%02X:%s' % (nid,self[nid].name) )
        else:
          lst.append( '%02X:<?>' % nid )
      print "%.2g:" % (now()-t0),", ".join(lst)
      sleep(1)

  def add( self, *modules ):
    """
    Add the specified modules (as returned from .newModuleIX()) 
    """
    for mod in modules:
      progress("Adding %s %s" % (mod.__class__.__name__,mod.name) )

      ##V: How to properly do this?
      assert isinstance(mod,Module) or isinstance(mod, pololu2_vv.PololuServoModule) or isinstance(mod, hitec.HitecServoModule)
      self.at._add(mod.name, mod)
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

  def iterhwaddr( self ):
    nids = self.keys()
    nids.sort()
    for nid in nids:
      for index in self[nid].iterhwaddr():
        yield Cluster.build_hwaddr( nid, index )
  
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
    self.p.update()
    s = set( ( nid 
      for nid,(ts,_) in self.p.heartbeats.iteritems()
      if ts + limit > t0 ) )
    return s

  @staticmethod
  def build_hwaddr( nid, index ):
    "build a hardware address for a property from node and index"
    return "%02x:%04x" % (nid,index)
  
  REX_HW = re.compile("([a-fA-F0-9]{2})(:)([a-fA-F0-9]{4})")
  REX_PROP = re.compile("([a-zA-Z_]\w*)(/)((?:[a-zA-Z_]\w*)|(?:0x[a-fA-F0-9]{4}))")
  REX_ATTR = re.compile("([a-zA-Z_]\w*)(/@)([a-zA-Z_]\w*)")
    
  @classmethod
  def parseClp( cls, clp ):
    """
    parse a class property name into head and tail parts
    
    use this method to validate class property name syntax.
    
    OUTPUT: kind, head, tail
        where kind is one of ":", "/", "/@"        
    """
    m = cls.REX_HW.match(clp)
    if not m: m = cls.REX_PROP.match(clp)
    if not m: m = cls.REX_ATTR.match(clp)
    if not m:
      raise ValueError("'%s' is not a valid cluster property name" % clp )
    return m.group(2),m.group(1),m.group(3)
        
  def modOfClp(self, clp ):
    """
    Find the module containing a given property
    """
    kind,head,tail = self.parseClp(clp)
    # Hardware names
    if kind==":":
      return self[int(head,16)]
    # Property or Attribute
    if kind[:1]=="/":
      return getattr( self.at, head )
    
  def _getAttrOfClp( self, clp, attr ):
    """(private)
    Obtain python attribute of a cluster property identified by a clp
    """
    kind,head,tail = self.parseClp(clp)
    # Hardware names
    if kind==":":
      nid = int(head,16)
      index = int(tail,16)
      try:
        od = self[nid].od
      except KeyError:
        raise KeyError("Unknown node ID 0x%02x" % nid)
      if od is None:
        raise ValueError("Node %s (ID 0x%02x) was not scanned for properties. Use get_od() or populate(...,walk=1) " % (self[nid].name,nid))
      try:
        return getattr(od.index_table[index],attr)
      except KeyError:
        raise KeyError("Unknown Object Dictionary index 0x%04x" % index)
        
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

class MemAt0x1010( object ):
  GIO_R_ADDR = 0x1011
  GIO_R_VAL = 0x1012
  GIO_W_ADDR = 0x1021
  GIO_W_VAL = 0x1022

class MemAt0x1000( object ):
  GIO_W_ADDR = 0x1001
  GIO_R_VAL = 0x1004
  GIO_R_ADDR = 0x1003
  GIO_W_VAL = 0x1002

class MemIxMixin( object ):
  """
  Module mixin class implementing memory interface

  This adds mem_read and mem_write methods, and a get_mem 
  """
       
  def mem_write( self, addr, val ):
     "Write a byte to a memory address in the module's microcontroller"
     self.pna.set( self.GIO_W_VAL, "B", val )
     self.pna.set( self.GIO_W_ADDR, "H", addr )

  def mem_read( self, addr ):
     "Read a memory address from the module's microncontroller"
     self.pna.set( self.GIO_R_ADDR, "H", addr )
     return self.pna.get_sync( self.GIO_R_VAL, "B" )
     
  def mem_getterOf( self, addr ):
     "Return a getter function for a memory address"
     # Create the closure
     def getter():
       return self.mem_read(addr)
     return getter
     
  def mem_setterOf( self, addr ):
     "Return a setter function for a memory address"
     # Create the closure
     def setter(val):
       return self.mem_write(addr,val)
     return setter
    
class SensorModule( Module ):
  """Abstract superclass for all sensor module classes
  
  Implements the .pins member, used to coordinate between ports regarding 
  pin usage.
  
  """
  def __init__(self, nid, typecode, pna):
      Module.__init__(self, nid, typecode, pna)
      self.pins = {}

class MCU_Port( object ):
  """Abstract superclass representing all MCU ports -- used or unused
  """
  def __init__(self,pins,**kw):
    """
    INPUTS:
      keyword parameters are assigned as attributes
    """
    for k,v in kw.iteritems():
      if hasattr(self,k):
        raise AttributeError("Attribute '%s' already exists" % k)
      setattr(self,k,v)
    self.need_pins = set(pins)
    self.active = False
  
  def use(self, pinass, pins=None):
    """
    INPUT:
      pinass -- dict -- table of pin assignments mapping pins to the port 
        object owning them
      pins -- set or None -- set of pins to actually take over; uses port 
        defaults if not specified
    OUTPUT:
      updates the value of pinass to reflect newly used pins
    """
    if pins is None:
      pins = self.need_pins
    used = set(pinass.keys())
    if not used.isdisjoint(pins):
      raise IndexError("Needed pins %s are not free" 
        % (used.intersection(pins) ) )
    for p in pins:
      pinass[p] = self
    self.active = True
    
class Pic18GPIO( MCU_Port ):
  """Concrete class representing a GPIO port on a PIC18 MCU
  
  Usage: assume m.gpio_A is a Pic18GPIO instance, and m has .mcu, .mem, .pins
  >>> m.gpio_A.use( m.pins )
  >>> m.gpio_A.set_inputs( 0xF0 )
  >>> m.gpio_A.put( 3 )
  >>> print m.gpio_A.get()
  """
  def __init__(self,pins,**kw):
    """
    INPUTS:
      keyword parameters are assigned as attributes
    """
    MCU_Port.__init__(self,pins,**kw)
    self.imask = "<<GENERATE ERROR FOR UNINITIALIZED MASK>>"

  def set_inputs( self, mask ):
    """
    INPUT:
      mask -- byte -- bit-mask of bits to be used as inputs
    """
    assert self.active,"Must call use() to activate this port first"
    mask = int(mask)
    if mask<0 or mask>255:
      raise ValueError("Mask 0x%x out of range -- must be a byte" % mask)
    self.imask = mask
    self._set_tris(mask)
        
  def get(self):
    """
    Reads GPIO pins and returns the value.
    
    The value of non-input pins is undefined
    """
    assert self.active,"Must call use() to activate this port first"
    return self._get_port()

  def put(self,val):
    """
    Writes a byte to a GPIO port.
    
    Only bits that were declared as non-inputs may be used.
    """
    assert self.active,"Must call use() to activate this port first"
    return self._set_latch(val & ~self.imask)
  
  def setterForMask(self,mask):
    """
    Returns a setter function that writes only specified bits
    """
    def setter(val):
      try:
        val = (val & mask) | (self._get_port() & ~mask)
        self._set_latch(val & ~self.imask)
      except TypeError,err:
        raise TypeError("%s : did you forget to call set_input()?" % str(err))
    return setter
    
class Sensor_v16( SensorModule, MemIxMixin ):
  """ Concrete class implementing a sensor module based on the Pic18 mcu
  
  Use the .gpio_A .gpio_B .gpio_C members to use the ports in GPIO modes.
  
  Typical usage:
  >>> m.gpio_A.use(m.pins)
  >>> m.set_input(0)
  >>> import time
  >>> while True:
  >>>   m.gpio_A.put(255)
  >>>   time.sleep(0.2)
  >>>   m.gpio_A.put(0)
  >>>   time.sleep(0.2)
  """
  def __init__(self, nid, typecode, pna):
     SensorModule.__init__(self, nid, typecode, pna)
     MemIxMixin.__init__(self)
     self.mcu = P18F2680
     self.mem = MemInterface( self )
     self.gpio_A = Pic18GPIO([2,3,4,5,6,7,9,10],
       _get_port = self.mem_getterOf(self.mcu.PORTA),
       _set_latch = self.mem_setterOf(self.mcu.LATA),
       _set_tris = self.mem_setterOf(self.mcu.TRISA)    
       )
     self.gpio_B = Pic18GPIO(xrange(21,29),
       _get_port = self.mem_getterOf(self.mcu.PORTB),
       _set_latch = self.mem_setterOf(self.mcu.LATB),
       _set_tris = self.mem_setterOf(self.mcu.TRISB)
       )
     self.gpio_C = Pic18GPIO(xrange(11,19),
       _get_port = self.mem_getterOf(self.mcu.PORTC),
       _set_latch = self.mem_setterOf(self.mcu.LATC),
       _set_tris = self.mem_setterOf(self.mcu.TRISC)        
       )            

class GenericIO( Sensor_v16, MemAt0x1000):
  pass

class SensorNode_v06( Sensor_v16, MemAt0x1010):
  pass

class ServoModule( Module ):
  """
  Abstract superclass of servo modules. These support additional functionality 
  associated with position servos:

  .set_pos -- sets position of the module. -- units in 100s of degrees between 
    -9000 and 9000. This is a "safe" version, which only accepts legal values
  .set_pos_UNSAFE -- sets position of module, without any validity checking
  .get_pos -- reads current position (may be at offset from set_pos, even without load)
  .go_slack -- makes the servo go limp
  .is_slack -- return True if and only if module is slack
  """

  # (pure) Process message ID for set_pos process messages
  PM_ID_POS = None 

  # (pure) Magic value that makes the servo go slack
  POS_SLACK = 9001
  
  # (pure) Object Dictionary index of current "encoder" position 
  ENC_POS_INDEX = None
  
  # Upper limit for positions
  POS_UPPER = 9000
  
  # Lower limit for positions
  POS_LOWER = -9000
    
  def __init__(self, *argv, **kwarg):
    Module.__init__(self, *argv, **kwarg)
    self._attr=dict(
      go_slack="1R",
      is_slack="1R",
      get_pos="1R",
      set_pos="2W",
      set_pos_UNSAFE="2W"      
    )

  def go_slack(self):
    """
    Makes the servo go limp

    """
    self.set_pos_UNSAFE(self.POS_SLACK)

  def set_pos_UNSAFE(self, val):
    """
    Sets position of the module, without any validity checking
    
    INPUT:
      val -- units in 100s of degrees between -9000 and 9000
    """
    val = int(val)
    self.pna.send_pm(self.PM_ID_POS, 'h', val)
  
  def set_pos(self,val):
    """
    Sets position of the module, with safety checks.
    
    INPUT:
      val -- units in 100s of degrees between -9000 and 9000
    """
    self.set_pos_UNSAFE(crop(val,self.POS_LOWER,self.POS_UPPER))
  
    
  def get_pos_PM(self):
    """
    Gets the actual position of the module via process messages.

    OUTPUT:
      val -- list of positions -- units in 100s of degrees between -9000 and 9000
      time -- list of timestamps --
    """
    pass
    #TODO

  def get_pos(self):
    """
    Gets the actual position of the module
    """
    return self.pna.get_sync(self.ENC_POS_INDEX, 'h')

  def get_pos_async(self):
    """
    Asynchronously gets the actual position of the module
    """
    return self.pna.get_async(self.ENC_POS_INDEX)

  def is_slack(self):
    """
    Gets the actual position of the module
    """
    pos = self.pna.get_sync(self.POS_INDEX, 'h')
    return pos == self.POS_SLACK
    
class V1_3Module( ServoModule ):
  """
  Concrete subclass of Module for V1.3 modules.
  """
  PM_ID_POS = 0x100
  PM_ID_ENC_POS = 0x200
  ENC_POS_INDEX = 0x1051
  POS_INDEX = 0x1050

class V1_4Module( ServoModule ):
  """
  Concrete subclass of Module for V1.4 modules.
  """
  PM_ID_POS = 0x100
  PM_ID_ENC_POS = 0x200
  ENC_POS_INDEX = 0x1051
  POS_INDEX = 0x1050
    
class MotorModule( Module ):
  """
  Abstract superclass of motor modules. These support additional functionality 
  associated with Motor Modules:

  .set_speed -- sets speed of the module -- units are RPM, range is +/-200
  """

  # (pure) Process message ID for set_speed process messages
  PM_ID_SPEED = None 

  # (pure) Upper limit for speed
  SPEED_UPPER = None
  
  # (pure) Lower limit for speed
  SPEED_LOWER = None
  
  # RPM to module speak conversion factor (x * RPM = -9000:9000)
  RPM_CONVERSION = 45

  def __init__(self, *argv, **kwarg):
    Module.__init__(self, *argv, **kwarg)
    self._attr=dict(
      set_speed="2W",
      go_slack="1R",
      set_speed_UNSAFE="2W"
    )

  def set_speed(self,val):
    """
    Sets speed of the module, with safety checks.
    
    INPUT:
      val -- units in between SPEED_LOWER and SPEED_UPPER
    """
    self.set_speed_UNSAFE(crop(val,self.SPEED_LOWER,self.SPEED_UPPER))
    
  def set_speed_UNSAFE(self, val):
    """
    Sets speed of the module, without any validity checking
    
    INPUT:
      val -- units in RPM  
    
    Do not use values outside the range SPEED_LOWER to SPEED_UPPER
    """
    # the 45 converts from RPM to +/- 9000 range the module expects
    val = int(self.RPM_CONVERSION*val)
    self.pna.send_pm(self.PM_ID_SPEED, 'h', val)

  def go_slack(self):
    """
    Makes the module go "slack": power down the motor.
    """
    # We send message directly because don't want RPM_CONVERSION to multiply it
    self.pna.send_pm(self.PM_ID_SPEED, 'h', self.SPEED_SLACK)
    
class ICRA_Motor_Module( MotorModule ):
  """
  Concrete subclass of Motor modules.
  """
  PM_ID_SPEED = 0x100
  SPEED_UPPER = 200
  SPEED_LOWER = -200
  SPEED_SLACK = 0
  
class Motor_Module_V1_0_MM( MotorModule ):
  """
  Concrete subclass of Motor modules.
  """
  PM_ID_SPEED = 0x100
  SPEED_UPPER = 9000/36
  SPEED_LOWER = -9000/36
  SPEED_SLACK = 9001
  RPM_CONVERSION = 36
  VEL_COMMAND_INDEX = 0x1030
  VEL_FEEDBACK_INDEX = 0x1031
  REL_POS_COMMAND_INDEX = 0x1032
  BRAKE_COMMAND_INDEX = 0x1033
  REL_POS_VELOCITY_CAP_INDEX = 0x1034
  RPM_FEEDBACK_INDEX = 0x1035
  
  def set_rel_pos(self, val):
    """
    Commands a relative position.
    
    INPUT:
        val -- units in between -32767 and 32767 (degrees * 100)
    """
    self.pna.set(self.REL_POS_COMMAND_INDEX, 'h', crop(val,-32767,32767))
    
  def set_servo_speed_cap_RPM(self, val):
    """
    Sets the maximum servo speed (speed when given a position command)
    
    INPUT:
        val -- RPM in between SPEED_LOWER and SPEED_UPPER
    """
    self.pna.set(self.REL_POS_VELOCITY_CAP_INDEX, 'h', crop(val*self.RPM_CONVERSION,self.SPEED_LOWER,self.SPEED_UPPER))
  
  def set_brake(self, val):
    """
    Sets the amount that the motor leads are tied together, therefore, braking
    
    INPUT:
      val -- number between 0 and 9000 where 9000 is always braking and 0 is never
    """
    self.pna.set(self.BRAKE_COMMAND_INDEX, 'h', crop(val,0,9000))
    
  def get_speed(self):
    """
    Gets the actual speed of the module in RPM
    """
    return self.pna.get_sync(self.RPM_FEEDBACK_INDEX, 'h')

  def get_speed_async(self):
    """
    Asynchronously gets the actual speed of the module in RPM
    """
    return self.pna.get_async(self.RPM_FEEDBACK_INDEX)
    
class IR_Node_Atx( Module, MemAt0x1010, MemIxMixin):
  """
    - IR Module Subclass. Has basic 'Module' functionality
        and should at some point have full memory access.
    - Will hopefully eventually have functions to deal with IR 
        communication and topology mapping. 
  """
  def __init__(self, nid, typecode, pna):
     Module.__init__(self, nid, typecode, pna)
     MemIxMixin.__init__(self)
     self.mcu = None
     self.mem = MemInterface( self )

#Register module types:
Module.Types['V1.3'] = V1_3Module
Module.Types['V1.4'] = V1_4Module
Module.Types['mm'] =ICRA_Motor_Module
Module.Types['V1.0-MM'] =Motor_Module_V1_0_MM
Module.Types['GenericIO'] =GenericIO
Module.Types['Sensor0.6'] =SensorNode_v06
Module.Types['Sensor0.2'] =SensorNode_v06
Module.Types['V0.1-ATX'] = IR_Node_Atx
Module.Types['PolServoModule'] = pololu.ServoModule
Module.Types['HitecServoModule'] = hitec.ServoModule
Module.Types['HitecMotorModule'] = hitec.MotorModule
Module.Types['Dynamixel-006b'] = dynamixel.EX106Module
Module.Types['Dynamixel-0040'] = dynamixel.RX64Module

