
"""
File joy.remote contains the remote.Source and remote.Sink Plan 
subclasses. These Plan-s may be used to relay remote JoyApp events 
between processes that may even be running on different machines

"""

from plans import Plan
from joy.events import JoyEvent, describeEvt, TIMEREVENT, KEYUP, KEYDOWN
from socket import socket, AF_INET, SOCK_DGRAM, error as SocketError
from errno import EAGAIN
from json import loads as json_loads, dumps as json_dumps
from pygame.event import post as pygame_event_post
from loggit import progress

# Default UDP port used for communications
DEFAULT_PORT = 0xBAA

class Sink( Plan ):
  """
  Concrete class remote.Sink
  
  This Plan subclass may be used to relay via UDP JoyApp events (including all pygame events) from a remote.Source running in 
  another JoyApp, potentially on another machine.
  
  Events are serialized into a JSON string and sent over separate UDP 
  packets. TIMEREVENTS are ignored.
  """ 
  DEFAULT_BINDING = ('0.0.0.0',DEFAULT_PORT)
  
  def __init__( self, app, bnd=DEFAULT_BINDING, rate=100, convert=None ):
    """
    Attributes:
      bnd -- 2-tuple -- binding address for socket, in socket library format
      rate -- integer -- maximal number of events processed each time-slice
      sock -- socket / None -- socket only exists while plan is running
      convert -- callable -- convert incoming event dictionaries.
        Allows remote joysticks/keys to be remapped. Returns None if
        event should be ignored
        By default, only KEYUP and KEYDOWN events are allowed 
    """
    Plan.__init__(self,app)
    self.bnd = bnd
    self.sock = None
    self.rate = rate
    if convert is None:
      def default_convert( dic ):
        tc = dic['type_code']
        if tc == KEYUP or tc == KEYDOWN:
          return dic
        return None
      self.convert = default_convert
    else:
      assert callable(convert)
      self.convert = convert
  
  def onStart( self ):
    self.sock = socket( AF_INET, SOCK_DGRAM )
    self.sock.bind(self.bnd)
    self.sock.setblocking(False)
  
  def onStop( self ):
    if self.sock:
      self.sock.close()
    self.sock = None
      
  def setSink( self, bnd ):
    """
    Set the socket binding address. 
    
    Will take effect next time the plan is started
    """
    self.bnd = bnd

  def onEvent( self, evt ):
    if evt.type != TIMEREVENT:
      return
    for k in xrange(self.rate):
      try:
        # Create event from the next packet
        pkt = self.sock.recv(1024)
        dic = json_loads(pkt)
        dic = self.convert(dic)
        # If converter dropped event --> next
        if not dic:
          continue
        # Put event on event queue
        nev = JoyEvent( **dic )
        #DBG progress('Remote event:'+str(nev))
        pygame_event_post(nev)
      #
      except SocketError,err:
        # If socket is out of data --> we're done
        if err.errno == EAGAIN:
          break
        raise
      #
      except ValueError,ve:
        # Value errors come from JSON decoding problems. 
        # --> Log and continue
        progress('Received bad UDP packet: %s' % repr(pkt))
        continue
    return False

class Source( Plan ):
  """
  Concrete class remote.Source
  
  This Plan subclass may be used to relay JoyApp events (including all
  pygame events) to a remote.Sink running in another JoyApp via UDP.
  This allows a controller in one JoyApp to control a client in a 
  remote JoyApp connected via an IP network, potentially running on 
  another host altogether.
  
  Events are serialized into a JSON string and sent over separate UDP 
  packets. TIMEREVENTS are ignored.
  """ 
  DEFAULT_SINK = ('localhost',DEFAULT_PORT)
  
  def __init__( self, app, dst=DEFAULT_SINK ):
    """
    Attributes:
      dst -- 2-tuple -- destination address for packets, in socket library format
      sock -- socket / None -- socket only exists while plan is running 
    """
    Plan.__init__(self,app)
    self.dst = dst
    self.sock = None
  
  def onStart( self ):
    self.sock = socket( AF_INET, SOCK_DGRAM )
    self.sock.bind(('0.0.0.0',0))
  
  def onStop( self ):
    if self.sock:
      self.sock.close()
    self.sock = None
      
  def setSink( self, dst ):
    """
    Set the sink address. Will take effect immediately
    """
    self.dst = dst

  def onEvent( self, evt ):
    if evt.type == TIMEREVENT:
      return False
    dic = describeEvt( evt, parseOnly = True )
    dic['t'] = self.app.now
    jsn = json_dumps(dic, ensure_ascii = True )
    self.sock.sendto( jsn, self.dst )
    #DBG progress( "Sending '%s' to %s:%d" % ((jsn,)+self.dst) )
    return False

