"""
FILE: polowixel.py

Architecture for a wixel interface to a pololu controller operating
continuous rotation servos with position feedback.

"""
from __future__ import division
from ckmodule import AbstractServoModule, AbstractNodeAdaptor, progress
from struct import unpack, error as struct_error
from pololu import (
  Bus as Bus0,
  Protocol as Protocol0,
  ProtocolNodeAdaptor as ProtocolNodeAdaptor0
)


class Bus( Bus0 ):
    def __init__(self, *arg, **kw):
        Bus0.__init__(self,*arg,**kw)
        self.buf = []
        self.n = 0
        self.itermax = 14

    def update(self, t=None):
        #progress('bus update')
        """
        update buffer by executing up to self.itermax read operations
        on the serial bus. Reading stops if no data is available
        """
        for n in xrange(self.itermax):
          #if not self.ser.inWaiting():
          #  return
          w = self.ser.read()
          self.buf.append(w)
          self.n += len(w)
          #progress("%r" %self.buf)

    def read(self,l):
        #progress( 'bus read' )
        """
        Read l characters from the serial bus after the next sync bytes,
        in an atomic read operation. If the data isn't available, read nothing
        """
        if not self.buf:
          progress("no data")
          return # no data
        if len(self.buf)>1:
          self.buf = ["".join(self.buf)]
        b = self.buf[0]
        s = b.find(b'\xff\xff')
        if s<0:
          progress("no sync")
          return # no sync bytes
        if self.n<s+2+l:
          res = ''
        else:
          res = self.buf[0][s+2:s+l+2]
          self.buf = [self.buf[0][s+4+l:]]
          self.n -= s+2+l
        return res

class Protocol( Protocol0 ):
    def __init__(self, *arg, **kw):
        kw.update(bus=Bus())
        if kw.has_key('posmap'):
          assert len(kw['posmap']) # is a sequence type
          self.posmap = kw['posmap']
          del kw['posmap']
        else:
          raise RuntimeError("Need a posmap from readings to nid")
        Protocol0.__init__(self,*arg,**kw)
        self.count = 6

        #progress('bus %r' % self.bus)

    def update(self,t=None):
        #progress( 'P update' )
        # Superclass update
        Protocol0.update(self)
        # trying to fix the bus issue
        # pull data from serial, if there is any
        self.bus.update(t)
        # Try to read positions of servos
        l = self.bus.read(self.count * 2)
        if not l:
          return
        # Push position into PNAs
        try:
          for ii,val in enumerate(unpack("h"*len(self.posmap),l)):
            nid = self.posmap[ii]
            if self.pnas.has_key(nid):
              self.pnas[nid].feedPos(val)
        except struct_error, er:
            progress("Error unpacking message ('%s', %r)" % (er,l))
            
    def generatePNA(self, nid):
        """
        Generates a pololu.ProtocolNodeAdaptor, associating a pololu protocol with
        a specific node id and returns it
        """
        pna = ProtocolNodeAdaptor(self, nid)
        self.pnas[nid] = pna
        return pna

class ProtocolNodeAdaptor( AbstractNodeAdaptor ):
  """
  Utilizes the protocol along with nid to create an interface for a
  specific module
  """
   # MiniSSCII Protocol Sync Value (must ALWAYS be 0xFF) specified by Pololu Documentation
  MINISSC2_BYTE = 0xFF

  # Compact Protocol Sync Value (must be 0x9F)
  COMPACT_BYTE = 0x8F

  # Pololu Protocol Sync Value (must be 0xAA)
  # This is also to initialize the Maestro to begin receiving commands using the PololuProtocol
  POLOLU_BYTE = 0xAA

  ##V: Need to put this in a proper location
  SLACK_MESSAGE = 0

  def __init__(self,*arg,**kw):
    self.rawpna = ProtocolNodeAdaptor0(*arg,**kw)
    self.nid = self.rawpna.nid
    self.go_slack = self.rawpna.go_slack
    self.lb = 600
    self.ub = 2800
    self.relpos = 0

  def feedPos(self,pos):
    #progress( 'feed pos' )
    """
    Feed a position reading from
    """
    # Update calibration range
    if pos > self.ub:
        self.ub = pos+1
    if pos < self.lb:
        self.lb = pos-1
    # Position as a fraction of calibrated range
    p = (pos - self.lb)/float(self.ub - self.lb)*360
    self.relpos = p

  def get_relpos(self):
    return self.relpos

  def sendCmd( self, cmd ):
    #progress('cmd send %r ' % self.nid)
    self.rawpna.p.send_cmd( self.MINISSC2_BYTE, self.nid, (cmd,) )
    #progress('cmd %r' % cmd)

  def get_typecode( self ):
    return "PoloWixelModule"

class ServoModule( AbstractServoModule ):
  """
  ServoModule Class has the basic functionality of ServoModules, with some exceptions listed below:

  - Pololu Modules cannot get_pos or is_slack
  - The Pololu Device allows for:
  - servo parameter settings
  - set speed
  - set neutral
  and various options for setting positions. We currently only use absolute
  positions however. For more information refer to the Pololu User Manual
  on p.6
  """
  MAXRPM = 140

  def __init__(self, node_id, typecode, pna, *argv, **kwarg):
    AbstractServoModule.__init__(self, node_id, typecode, pna, *argv, **kwarg )
    self._attr.update(
      go_slack="1R",
      set_pos="2W",
      get_pos="1R",
      is_slack="1R"
      )

    ##V: How should these be initialized? I just initialize them in set_pos / go_slack for now
    self.slack = None; # Initialized to True or False
    self.pos = None; # Initialized to start position of module
    self.punch = 7
    self.goalPos = 0
    self.Kp = 5.0

  def set_rpm(self, target):
    #progress( 'set rpm' )
    """
    Sends a command to the Pololu device over serial via Protocol.send_cmd()

    INPUT:
    target -- float -- rate of rotation
    data -- int -- the payload data to send to the module
    """
    if abs(target)>self.MAXRPM:
        raise ValueError("RPM value must be between 0 and %d" % self.MAXRPM)
    if target>0:
        sgn = 1
    elif target<0:
        sgn = -1
    else:
        sgn = 0
    # Essentially passes on the call to the protocol, includes the nid
    cmd = int( 127 + sgn*self.punch + float(target)*(127-sgn*self.punch)/self.MAXRPM )
    #progress("%g" % (float(target)*(127-self.punch)/self.MAXRPM))
    #progress('t %r' % target)
    #progress("%r -> %s" % (target,cmd))
    self.pna.sendCmd(cmd)

  def update( self, t ):
    #progress('servo update')
    """
    Perform a periodic update for controlling motor position

    Implements a proportional feedback controller
    """
    self.pos = self.pna.get_relpos()
    #progress( "ServoModule.update %r,%r" % (self,self.pos))
    if not self.Kp or self.slack:
      return
    # Implement proportional feedback
    er = (self.pos - self.goalPos + 0.5) % 1.0 - 0.5
    #er = (1 - self.goalPos + 0.5) % 1.0 - 0.5
    self.set_rpm( self.Kp * er )

  def set_pos( self, pos ):
    self.goalPos = pos % 1.0

  def is_slack(self):
    """
    Returns true if the module is slack, none if go_slack has not been called yet.

    WARNING: This function does NOT actually read states from the pololu device, returns an attribute that is updated by calls to set_pos and go_slack. If any external communications fail, then this function may report incorrect states
    """
    return self.slack

  def get_pos(self):
    """
    Return the most recent position recorded from module.

    NOTE: before the module has made a complete rotation, this result is highly
    suspect...
    """
    return self.pos

  def get_pos_async(self):
    """
    Return the most recent position recorded from module.

    NOTE: before the module has made a complete rotation, this result is highly
    suspect...
    """
    return self.pos

  def go_slack(self):
    """
    Equivalent of setting a ServoModule slack. This is referred to as "off"
    as specified in the Pololu User Manual under the Command 0
    """
    # Send the command via the PNA
    self.pna.go_slack()
    # Module should now be slack
    self.slack = True
