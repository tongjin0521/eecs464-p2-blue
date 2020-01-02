"""
module ckbot.port2port

This module provides point-to-point connectivity tools for the rest of
the ckbot classes. In particular, it encapsulates the differences
between serial ports in Linux, OS X and other common operating
environments, as well as offering a UDP transport and an xbee wrapper
for serial links.

All connections provided by this module are subclasses of Connection
class.
"""

from serial import Serial
from socket import socket, error as SocketError, AF_INET, SOCK_DGRAM,SOL_SOCKET,SO_REUSEADDR
from glob import glob as GLOB_GLOB
from sys import platform, stderr
from os import sep
from json import loads as json_loads
from time import time as now
from errno import EAGAIN
# Handle windows
try:
  import _winreg as winreg
except:
  winreg = None

class Connection( object ):
  """
  Abstract superclass of all Connection objects
  """

  def open( self ):
    """Make the connection active"""
    pass

  def isOpen( self ):
    """Test if connection is open"""
    return False

  def flush(self):
    pass

  def inWaiting(self):
    return 0

  def write( self, msg ):
    """(pure) Attempt to write the specified message through the
    connection. Returns number of byte written
    """
    raise RuntimeError("Pure method called")

  def read( self, length ):
    """(pure) Attempt to read the specified number of bytes.
    r
    Return number of bytes actually read
    """
    return 0

  def close( self ):
    """Disconnect; further traffic may raise an exception
    """
    pass

  def reconnect( self, **changes ):
    """Try to reconnect with some configuration changes"""
    pass

class SerialConnection( Serial, Connection ):
  """
  Concrete Connection subclass representing a serial port
  """
  def __init__(self, glob=None, *args, **kw):
    # Try to resolve user globs
    port_path = None
    if glob is not None:
      port_path = (GLOB_GLOB(glob)+[None])[0]
    if port_path is None:
      # Determine port name for current operating system
      plat = platform.lower()
      if "linux" in plat: # Linux
        port_path = (GLOB_GLOB("/dev/ttyUSB*")+[None])[0]
      elif "win32" in plat: # Windows
        # Grab first serial port from windows machine
        path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
        port_path = winreg.EnumValue(key,0)[1]
      elif "darwin" in plat: # Mac
        # Grab the first usbserial, failing that the first usbmodem
        port_path = (GLOB_GLOB("/dev/tty.usbserial*")+GLOB_GLOB("/dev/tty.usbmodem*")+[None])[0]
      else: # Unhandled OS
        raise IOError('Unknown OS -- cannot auto-configure serial')

    if port_path is None:
      raise IOError("No serial port found and no glob hint given")
    Serial.__init__(self, port_path, *args, **kw)
    Connection.__init__(self)
    if self.isOpen():
      self.path = port_path
    else:
      self.path = "<<COULD-NOT-OPEN>>"

  """Since pySerial decided to break things, lets un-break them"""
  if not hasattr(Serial,'getSupportedBaudRates'):
     def getSupportedBaudrates(self):
        if type(self.BAUDRATES[0]) is int:
          return list(self.BAUDRATES)
        return [ (b,None) for b in self.BAUDRATES ]

  def __repr__(self):
    r = object.__repr__(self)
    return "%s %s %d:%d%s>" % (r[:-1],self.path, self.baudrate,self.stopbits,self.parity )

def newConnection( spec ):
  """Factory method for creating a Connection from a string or dictionary

  Formats:
  udp={dst="host", dport=<port>, sport=<sport>} -- set up a UDP
    connection.
  tty={glob=<glob>, baudrate=<baud>, stopbits = <n>, parity = 'N'|'E'|'O', timeout=0.5 } -- set up a tty connection

  In dictionary format, the Connection type is given by the TYPE key, e.g.
    dict( TYPE='tty', baudrate=9600 )
  other: assumed to be a Serial device glob pattern
  """
  if type(spec) is str:
    spl = spec.split("=",1)
    if len(spl)==1: # No '=' found --> legacy string is a serial device path
      # Just a plain string; fall back on legacy interpretation
      res = SerialConnection(spec)
      if not res.isOpen():
        raise IOError("Could not open '%s'" % spec)
      return res
    args = json_loads( spl[1] )
    T = spl[0]
  elif type(spec) is not dict:
    raise TypeError('spec must be a string or a dictionary')
  else:
    args = spec.copy()
    T = args['TYPE']
    del args['TYPE']

  if T.lower() == 'udp':
    res = UDPConnection( **args )
  elif T.lower() == 'tty':
    if 'glob' not in args:
      res = SerialConnection(**args)
      if not res.isOpen():
        raise IOError("Could not open default port")
    else:
      kw = args.copy()
      g = kw['glob']
      del kw['glob']
      res = SerialConnection(g,**kw)
      if not res.isOpen():
        raise IOError("Could no open '%s'" % g)
  else:
    raise ValueError('Unknown Connection type %s' % repr(T))
  # new connection is ready, store the spec used to create it
  args['TYPE']=T
  res.newConnection_spec = args
  return res
  #

class UDPConnection( Connection ):
  def __init__(self,*arg,**kw):
    cfg = dict(interface='0.0.0.0',port=6666,reuse=True,maxlen=1024)
    self.cfg = cfg
    self.sock = None
    self.peer = None
    self.reconnect(**kw)

  def open( self ):
    self.sock = socket( AF_INET, SOCK_DGRAM )
    self.sock.setsockopt( SOL_SOCKET, SO_REUSEADDR, 1 if self.cfg['reuse'] else 0 )
    self.sock.bind((self.cfg['interface'],self.cfg['port']))
    self.sock.setblocking(0)
    self.rxq = []
    self.rxlen = 0

  def getsockname(self):
    """Return socket name of local socket"""
    return self.sock.getsockname()

  def isOpen( self ):
    """Test if connection is open"""
    return bool(self.sock)

  def inWaiting(self):
    self._readSock()
    return self.rxlen

  def write( self, msg ):
    """Attempt to write the specified message through the
    connection. Returns number of byte written
    """
    # Send to dst if set, otherwise send back to our peer
    tgt = self.cfg.get('dst',self.peer)
    len = 0
    if tgt is None:
        raise RuntimeError("Unable to send -- no UDP target set")
    else:
        len = self.sock.sendto(msg,tgt)
    return len

  def read( self, length ):
    """
    Attempt to read the specified number of bytes.
    Return at most that many bytes
    """
    self._readSock()
    if not self.rxq:
        return b""
    _,pkt,_ = self.rxq.pop(0)
    self.rxlen -= len(pkt)
    return pkt[:length]

  def msgIter(self):
      """
      Iterator returning triples of ts,msg,src
        ts -- recieve timestamp
        msg -- the message
        src -- UDP source address of the packet
      If no data is available, then pkt is None
      """
      while True:
          self._readSock()
          if self.rxq:
              ts,pkt,src = self.rxq.pop(0)
              self.rxlen -= len(pkt)
              yield ts,pkt,src
          else:
              yield 0,None,('0.0.0.0',0)

  def _readSock(self,rate=50):
    """(private)
    Process at most 'rate' packets into the queue
    """
    mxl = self.cfg['maxlen']
    while rate>0:
        try:
            # Create event from the next packet
            pkt,src = self.sock.recvfrom(mxl)
            ts = now()
        except SocketError as err:
            if err.errno != EAGAIN:
                raise
            # Socket had no data
            return
        self.peer = src
        self.ts = ts
        self.rxq.append((ts,pkt,src))
        self.rxlen += len(pkt)
        rate -= 1
    return

  def close( self ):
    """Disconnect; further traffic may raise an exception
    """
    if self.sock:
        self.sock.close()
    self.sock = None

  def reconnect( self, **changes ):
    """Try to reconnect with some configuration changes"""
    self.close()
    self.cfg.update(changes)
    self.open()
