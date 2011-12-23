"""
The CKBot.dynamixel python module provides classes used to communicate with CKBot robot modules connected the [Robotis] Dynamixel Single-Line Serial or Half-Duplex RS485 Busses (rather than CAN or Hitec HMI serial bus) using a protocol similar to the Robotics Bus Protocol. These classes are primarily used to interface with the CKBot.logical python module found in the Modlab CKBot repository. For more information on
the low-level interface, Robotis Electronic Manual http://support.robotis.com/en/


Main uses of this module:
(*) control CKBot robot modules (specifically the servos) connected to a the Dynamixel Serial or RS485 bus
(*) mimics the behaviour of the CAN Bus, except using the Dynamixel Bus  as a communications channel

Example 1 - hitec robot module only:
  #>>> import logical
  #>>> nodes = [0x01, 0x06]
  #>>> bus = hitec.Bus()
  #>>> p = hitec.Protocol(bus = bus, nodes = nodes)
  #>>> p.send_cmd([0x02,7,0])

Example 2 - Integrate with CKBot.logical python module:
  >>> import logical
  >>> nodes = [0x01, 0x06, 0x03]
  >>> bus = dynamixel.Bus("/dev/tty.usbserial-A600e1fL") 
  >>> p = dynamixel.Protocol(bus = bus, nodes = nodes)
  >>> c = logical.Cluster(p)
  >>> c.populate(3,{0x01:'head', 0x06:'mid', 0x03:'tail'})
  >>> c.at.head.set_pos(1000)


Both examples sets the position of the 0xD1 robot module, to 1000 (10 degrees from the  neutral position)
"""

from time import time, sleep
from sys import platform as SYS_PLATFORM, stdout
from struct import pack, unpack
from array import array
from serial import Serial
from subprocess import Popen, PIPE
from glob import glob
from random import uniform

from ckmodule import Module, AbstractNodeAdaptor, AbstractProtocol, progress

def crop( val, lower, upper ):
  return max(min(val,upper),lower)

class Dynamixel( object ):
  """ Namespace containing useful constants and functions"""

  CMD_PING = 0X01
  CMD_READ_DATA = 0X02
  CMD_WRITE_DATA = 0X03
  CMD_REG_WRITE = 0X04
  CMD_ACTION = 0X05
  CMD_SYNC_WRITE = 0X83

  # Currently just a bunch of address variables
  # EEPROM
  ADDR_MODEL = 0X00
  ADDR_VERSION = 0X02
  ADDR_ID = 0X03
  ADDR_BAUD = 0X04
  ADDR_RET_DELAY = 0X05
  ADDR_CW_ANGLE_LIMIT = 0X06
  ADDR_CCW_ANGLE_LIMIT = 0X08
  ADDR_DRIVE_MODE = 0X0A
  ADDR_MAX_TEMP = 0X0B
  ADDR_MIN_VOLTAGE = 0X0C
  ADDR_MAX_VOLTAGE = 0X0D
  ADDR_MAX_TORQUE = 0X0E
  ADDR_STATUS = 0X10
  ADDR_ALARM_LED = 0X11
  ADDR_ALARM_SHUTDOWN = 0X12
  #RAM
  ADDR_TORQUE_EN = 0X18
  ADDR_LED = 0X19
  ADDR_CW_COMPLIANCE_MARGIN = 0X1A
  ADDR_CCW_COMPLIANCE_MARGIN = 0X1B
  ADDR_CW_COMPLIANCE_SLOPE = 0X1C
  ADDR_CCW_COMPLIANCE_SLOPE = 0X1D
  ADDR_DESIRED_POSITION = 0X1E
  ADDR_DESIRED_SPEED = 0X20
  ADDR_TORQUE_LIMIT = 0X22
  ADDR_PRESENT_POSITION = 0X24
  ADDR_PRESENT_SPEED = 0X26
  ADDR_PRESENT_LOAD = 0X28
  ADDR_PRESENT_VOLTAGE = 0X2A
  ADDR_PRESENT_TEMPERATURE = 0X2B
  ADDR_REGISTERED_INSTRUCTION = 0X2C
  ADDR_MOVING = 0X2E
  ADDR_LOCK = 0X2F
  ADDR_PUNCH = 0X30
  ADDR_PRESENT_CURRENT = 0X38

  MEM_LEN = 0x39
  SYNC = '\xff\xff'
  
  
##V: These need to be fixed : U ??
class BusError( RuntimeError ):
  pass

class ProtocolError( RuntimeError ):
  pass

class Bus( object ):
  """
  Concrete class that provides the functionality
  needed to send messages to a Dynamixel Servo
  over their serial/RS485 connection. 
  
  It is responsible for correctly formatting certain inputs
  to dynamixel-understandable formats as documented in the Dynamixel
  documentation
  """
  """
  Utilizes the protocol along with nid to create 
  an interface for a specific module
  """
  def __init__(self, port = None):
    """ 
    Initialize hitec bus
    
    INPUT: 
    port -- serial port string, or None to autoconfig  
    
    ATTRIBUTES:
    ser -- serial handle 
    """
      # Set up serial with parameters defined by HiTech
    if port is None:
      port = self._auto_serial()
    self.ser = Serial(port)
    self.ser.baudrate = 1000000
    self.ser.stopbits = 1
    self.ser.parity = 'N'
    self.ser.timeout = 0.1
    self.ser.open()
    # Initialize receive buffer
    self.rxbuf = ''
    self.rxln = None
    ###self.stats = dict(out=0,inp=0,sync=0,retry=0,frame=0,fail=0,empty=0)

  @staticmethod
  def _auto_serial():
    # Determine port name for current operating system
    port_num = 0
    plat = SYS_PLATFORM.lower()        
    if "linux" in plat: # Linux
      port_path = (glob("/dev/ttyUSB*")+[None])[0]        
    elif "win32" in plat: # Windows
      port_path = "7"          
    elif "darwin" in plat: # Mac
      port_path = (glob("/dev/tty.usbserial*")+[None])[0]        
    else: # Unhandled OS
      raise IOError('Unknown OS -- cannot auto-configure serial')
    return port_path

  def close( self ):
    """
    Close communication with servo bus
    """
    self.ser.close()
    self.rxbuf = ''
    
  def reset( self ):
    """
    Reset communication with servo bus
    ... note: this can be useful when serial fails after a power reset
    """
    self.ser.close()
    self.ser.open()
    self.rxbuf = ''

  def read( self ):
    """ read a single frame off the serial or returns None
          note: is pretty hacky !!! needs work !!!
    """
    b = self.rxbuf
    print repr(b)
    ln = None
    chk = None
    while self.ser.inWaiting() or b:
      print ln
      print chk
      # Read in individual bytes until SYNC is found, then read in rest of packet
      # If there are bytes to be read then read them
      if self.ser.inWaiting():
        b = b + self.ser.read(1)
      # If not see if there are enough bytes in the current buffer
      else:
        if len(b) < 3:
          print 'not enough bytes'
	  self.rxbuf = b
          return None

      # Find first valid packet, throw out all preceding bytes
      if len(b) > 2:
        if b[:2] != Dynamixel.SYNC:
          print 'popping byte'
          b = b[1:]
          continue
      if len(b) > 4:
        ln = ord(b[3])
        # Check if length makes sense ... it cannot be greater than the 
 	# length of the control table ie: 57
        if ln > 57:
          b = b[1:]
          print 'wrong length'
          continue
        if len(b) < (ln+4):
          print 'b too small'
          self.rxbuf = b
          return None
        if len(b) >= (ln+4):
          chk_rx = ord(b[ln+3])
          pkt = unpack(str(len(b[2:(ln+3)])) + 'B', b[2:(ln+3)])
          chk = self._checksum(pkt)
          if chk_rx == chk:
            # Return packet as a nice dictionary
            self.rxbuf = b[(ln+4):] # Flush used bytes out of buffer
            ret = {'nid':pkt[0], 'len':pkt[1], 'err':pkt[2], 'params':pkt[3:]}
            return ret
          else:
            print 'bad chk'
            b = b[1:]
    return None

  def write( self, nid, cmd,  params=[], ln=None ): 
    """ 
    Formats the output message, transmits it
    INPUT:
    nid -- node id
    cmd -- command
    params -- parameter array
    RETURNS: formated output message 
    """
    if ln is None:
      ln = len(params) + 2
    chk = self._checksum([nid, ln, cmd] + params)
    pkt = pack('3B', nid, ln, cmd)
    pkt = Dynamixel.SYNC + pkt
    for c in xrange(0, len(params)):
      pkt += chr(params[c])
    pkt += chr(chk)
    self.ser.write(pkt)
    # Return packet as a nice dictionary
    ret = {'nid':nid, 'len':ln, 'cmd':cmd, 'params':params}
    #print repr(ret) DEBUG
    return ret
 
  def send_cmd_sync( self, nid, cmd, params=[], pause=0.05, retries=4, timeout=0.05):
    """Send a command in synchronous form, waiting for reply
      
    If reply times out -- retries until retry count runs out
    """
    for k in xrange(retries):
      at0 = self.write(nid, cmd, params)['nid']
      #print at0
      sleep(pause)
      at1 = False
      now = time()
      while not at1 and time()-now < timeout:
        rx = self.read()
        #print repr(rx)
        if rx is not None:
          at1 = rx['nid']
        if at0 == at1:
          return rx
    return None
   
  @classmethod 
  def _checksum(cls, pkt ):
    """ Compute the checksum for a Dynamixel Packet """
    chk = sum(pkt)
    if chk < 0xFF:
      chk = chk ^ 0xFF
    else:
      chk = (chk >> 8) ^ 0xFF
    return chk
  
class EEPROM_orMaybe_MEM( object ):
  """ Filler class for doing eeprom/memory operations """
  def __init__(self):
    progress("Not implemented, bother Uriah!!!")

class Protocol(AbstractProtocol):
  """
  TBD
  """
  def __init__(self, bus=None, nodes=None):
    """
    Initialize a dynamixel.Protocol
    
    INPUT:
    bus -- dyamixel.Bus -- serial/rs485 bus used to communicate with Dynamixel Servos
    nodes -- list -- list of hitec ids
    
    ATTRIBUTES:
    heartbeats -- dictionary -- nid : (timestamp, last message)
    pnas -- dictionary -- table of NodeID to ProtocolNodeAdapter mappings
    """
    AbstractProtocol.__init__(self)
    if bus is None:
      self.bus = Bus()
    else:
      self.bus = bus

    self.heartbeats = {}
    self.responses = {}
    self.pnas = {}
    self.ping_period = 3

    # !!! Implement this !!!
    #if nodes is None:
    #  progress("Scanning bus for nodes \n")
    #  nodes = self.scan()
    
    # FIX For now if nodes is None, then just assume that the user
    # wants to check for all nodes	
    if nodes is None:
      nodes = range(0, 254)

    for nid in nodes:
      self.generatePNA(nid)
    progress("Dynamixel nodes: %s\n" % repr(list(nodes)))

    # !!! Implement this !!!
    #def scan( self ):
    #  """ Do a low-level scan for all nodes """
      
  def write( self, *args, **kw ):
    """ Write out a command --> redirected to Bus.write """
    return self.bus.write( *args, **kw )

  def read( self ):
    """ Read a response --> redirected to Bus.read """
    return self.bus.read()
    
  def send_cmd_sync( self, *argv, **kw ):
    return self.bus.send_cmd_sync( *argv, **kw )

  def hintNodes( self, nodes ):
    n1 = set(nodes)
    n0 = set(self.pnas.keys())
    for nid in n0 - n1:
      # corrupt the pna, just in case someone has access to it
      self.pnas[nid].nid = "<<INVALID>>"
      del self.pnas[nid]
    for nid in n1-n0:
      self.generatePNA(nid)

  def generatePNA( self, nid ):
    """
    Generates a dynamixel.ProtocolNodeAdaptor, associating a dynamixel 
    protocol with a specific node id and returns it
    """
    pna = ProtocolNodeAdaptor(self, nid)
    self.pnas[nid] = pna
    return pna
    
  def _pingAsNeeded( self, now ):
    """(private)
    Periodically ping expected nodes to ensure that they are still alive
    """
    for nid,pna in self.pnas.iteritems():
      ts = self.heartbeats.get(nid,[0])[0]
      if (now - ts) > self.ping_period * uniform(0.7,0.9):
        pna.ping()

  def _completeResponses(self, now, timeout = 0.005, retries = 4):
    """(private)
    Handles the responses of incomplete messages
    - Re-requests if an incomplete message has timed out
    - Completes any request for a response that has arrived
    
    
    After server retries puts a ProtocolError in the result
    
    INPUT:
      now -- current time
      timeout -- retry rate in seconds
      retries -- number of retries
    OUTPUT:
      num_failed -- number of failed requests
    """
    """
    """
    num_failed = 0
    # Try to handle all current incomplete messages
    for nid in self.pnas.iterkeys():
      pna = self.pnas.get(nid)
      incm = pna._incm      
      #print incm
      # Check if there exist any incomplete messages to be handled
      if not incm:
        continue

      inc = incm.pop(0)
      # Check if current incomplete message has sent a write request,
      #  if not, write 
      if not inc.msg:
        msg = self.bus.write( inc.nid, inc.cmd, inc.params ) 
        inc.msg = msg
        incm.insert(0, inc)
        continue
    
      # Otherwise check if response exists for given nid
      if self.responses.has_key(nid):
        # If so, then check for valid timestamp
        ts, msg = self.responses.pop(nid)
        if now > inc.ts:
          #print msg
          inc.promise[:] = (msg,)
          continue
        # Otherwise it is an old response, wait for a new one
      
      # If a response didn't exist or was not new, then check if the 
      #  incomplete message hasn't timed out    
      if now - inc.ts > timeout:
        msg = inc.msg
        # If it has more retries, then retry and increment retries
        if inc.retries >= retries:
          inc.promise[:] = [ProtocolError(inc.nid)]
          num_failed += 1
          continue
        else:
          self.bus.write(msg['nid'], msg['cmd'], msg['params'], msg['len'])
          inc.ts = now
          inc.retries += 1      
      incm.insert(0, inc)

    return num_failed

  def _getResponses( self, now ):
    """
    read in the responses from the bus, update heartbeats and response dictionaries
    as needed 
    """
   # Only process for a timeslice at the longest
    timeslice = 0.05 # <- this seems pretty long -U
    num_responses = 0
    while time() - now < timeslice:
      msg = self.read()
      if msg is None:
        break
      # If the message is not none, then it indicates that there
      #  exists a live module for a given nid
      #  writes the timestamp and the error state of the node
      self.heartbeats[msg['nid']] = (now, msg['err'])
      # Check if message length is greater than 0, if so, then 
      #  add to dictionary that describes response for a given node
      if msg['len'] > 2:
        num_responses+=1
        self.responses[msg['nid']] = (now, msg)
    return num_responses
 
  def update( self ):
    """
    Complete any async requests and collect heartbeats
    """
    now = time()    
    # Read in responses from the bus
    num_responses = self._getResponses(now)
    # Process existing incomplete messages
    self._completeResponses(now)
    # Ping modules on the bus occasionally, to ensure that they are live
    self._pingAsNeeded(now)
    return        

  def request( self, nid, cmd, params=[], **kw ):
    """Checks if a given pna already has an incomplete message
         if there is still a pending incomplete message
         then append the request to the pna's incomplete messages
    
    return promise
    """
    # Check if there exists a pending request, else, pop None request
    # and handle next request
    pna = self.pnas[nid]
    incm = pna._incm
    if not incm:
      msg = self.bus.write( nid, cmd, params )
      inc = IncompleteMessage( nid, cmd, params, msg )
      incm.append(inc)
    else:
      inc = IncompleteMessage( nid, cmd, params )
      incm.append(inc)
    # Return current pending promise
    return inc.promise

class IncompleteMessage( object ):
  """ internal concrete class representing incomplete messages
  """
  def __init__(self, nid, cmd, params, msg=None, ts=None, retries=0 ):
    if ts is None:
      ts = time()
    self.nid = nid
    self.cmd = cmd
    self.params = params
    self.msg = msg
    self.ts = ts
    self.retries = retries
    self.promise = []
      
class ProtocolNodeAdaptor(AbstractNodeAdaptor):
  """
  Uses the protocol along with the nid to create an interface
  for a specific module
  """
  def __init__(self, protocol, nid = None):
    AbstractNodeAdaptor.__init__(self)
    self.p = protocol
    self.nid = nid
    self.eeprom = None  
    self._incm = [] # Queue of incomplete messages to be handled

  def get_typecode( self ):
    """
    Returns the name of a given module 
    !!! Note: this needs to be changed, I feel like it should say
              more about what the module actually is
    !!!
    """
    return "DynamixelModule"

  @classmethod
  def async_parse( cls, promise, barf=True ):
    """
    Parse the result of an asynchronous get.
    
    When barf is True, this may raise a deferred exception if the
    operation timed out or did not complete. Otherwise the exception
    object will be returned.
    """
    if not promise:
      raise ProtocolError("Asynchronous operation did not complete")
    dat = promise
    if barf and isinstance(dat, Exception):
      raise dat
    return dat[0]

  def get_async( self, addr_start, type='uint8' ):
    """
    This method get the value of a given set of memory from the 
    Dynamixel module in an asynchronous way.

    ... one could eventually image this communicating with a 
        dictionary that defines each address and its number of bytes
    
    INPUTS:
      addr_start -- start address in memory
      num_bytes -- number of bytes to read, if 1:uint8, 2:uint16
    """
    if type is 'uint8':
      num_bytes = 1
    elif type is 'uint16':
      num_bytes = 2
    else:
      raise ProtocolError("Invalid Type")

    return self.p.request( self.nid, \
                           Dynamixel.CMD_READ_DATA, \
                           [addr_start, num_bytes]
    )
                             
  def get_sync(self, addr_start, type='uint8', tic=0.005):
    """
    Synchronous wrapper on top of get_async
    
    Returns the get_async result
    """
    promise = self.get_async(addr_start, type)
    while not promise:
      sleep(tic)
      self.p.update()
    return self.async_parse(promise)

  def set( self, addr_start, lb=None, hb=None ):
    """
    A wrapper for Protocol.write.
      writes data to dynamixel servo memory
    """
    if hb is None:      
      self.p.write(self.nid, Dynamixel.CMD_WRITE_DATA, [addr_start, lb])
    else:
      self.p.write(self.nid, Dynamixel.CMD_WRITE_DATA, [addr_start, lb, hb])
    return addr_start

  def ping(self):
    """
    Send query to servo without having protocol expect a reply
    """
    self.p.write(self.nid, Dynamixel.CMD_PING)        
        

## Filler classes so that we can test Cluster    

class DynamixelModule( Module ):
  def __init__(self, node_id, typecode, pna, *argv, **kwarg):
    Module.__init__(self, node_id, typecode, pna)
    
    self.slack = False
    return

  def ping(self):
    return self.pna.ping()

  def get_led(self):
    val = self.pna.get_sync(Dynamixel.ADDR_LED)
    print repr(val)
    if val is None:
      print 'protocol error'

  def set_led(self, val):
    return self.pna.set(Dynamixel.ADDR_LED, val)
    

class ServoModule( DynamixelModule ):
  def __init__(self, node_id, typecode, pna, *argv, **kwarg):
    DynamixelModule.__init__(self, node_id, typecode, pna)
    return
    
