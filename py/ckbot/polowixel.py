"""
FILE: polowixel.py

Architecture for a wixel interface to a pololu controller operating
continuous rotation servos with position feedback.

"""
from __future__ import division
from warnings import warn
from struct import unpack, error as struct_error
from .ckmodule import AbstractServoModule, AbstractNodeAdaptor, progress, AbstractBusError
from .pololu import (
  Bus as pololu_Bus,
  Protocol as pololu_Protocol,
  ProtocolNodeAdaptor as pololu_ProtocolNodeAdaptor
)

DEFAULT_PORT = dict(
    TYPE='tty',
    glob="/dev/ttyACM*",
    baudrate=115200
)
DEBUG = {}

def _DBG(fun,fmt,*argv):
    """
    DEBUG assist function

    If fun is callable
        call with format and argv
    otherwise
        log message using format and argv

    USAGE:
       if key in self.DEBUG:
           _DBG(key,fmt,*argv)
    """
    if not callable(fun):
        progress(fmt % argv)
    else:
        fun(fmt,*argv)

class Bus( pololu_Bus ):
    """
    Concrete class Bus extending the pololu_Bus class

    This class adds an update function that pulls all data from the bus,
    and a read function that aligns to the \xFF\xFF sync byte pattern
    used in our wixel code
    """

    # Communications timeout
    TIMEOUT = 1.0

    def __init__(self, *arg, **kw):
        pololu_Bus.__init__(self,*arg,**kw)
        self.buf = b''
        self.n = 0
        self.lastUpdT = None
        self.t = None

    def update(self,t):
        self.t = t

    def read(self,l,skip=False):
        """
        Read an l character frame from the serial from after the next sync
        bytes, in an atomic read operation.

        When skip is true, tries to read the last frame of length l by skipping
        to the end of the buffer minus length of frame with l bytes

        If the requested data isn't available, returns None
        """
        w = self.ser.read(self.ser.inWaiting())
        if w:
            self.lastUpdT = self.t
        elif not (self.lastUpdT is None or self.t is None) and (self.t-self.lastUpdT)>self.TIMEOUT:
            raise AbstractBusError("Communication timed out")
        b = self.buf + w
        # If skip requested, and possible --> do it
        if skip and len(b)>(l+2):
            b = b[-l-2:]
        s = b.find(b'\xff\xff')
        if s<0:
          # Only report an error if sync search failed with bytes in buffer
          if len(b):
              progress("%s no sync (%d bytes)" % (repr(self),len(b)))
          self.buf = b
          return None # no sync bytes
        # If not enough bytes for atomic read --> return nothing
        if len(b)<s+2+l:
          self.buf = b
          return None
        # Read selected number of bytes
        res = b[s+2:s+l+2]
        self.buf = b[s+4+l:]
        self.n -= s+2+l
        return res

class Protocol( pololu_Protocol ):
    def __init__(self, *arg, **kw):
        port = dict(TYPE='TTY', glob="/dev/ttyACM*", baudrate=115200)
        port.update(kw.get('port',{}))
        kw.update(bus=Bus(port=port))
        pololu_Protocol.__init__(self,*arg,**kw)
        self.DEBUG=DEBUG

    def update(self, t = None):
        # Superclass update
        pololu_Protocol.update(self)
        # Bus update
        self.bus.update(t)
        # Read the latest update off of the bus
        l = self.bus.read(self.count * 2,skip=True)
        if l is None:
            return
        # Push position into PNAs
        try:
          msg = unpack("h"*self.count,l)
        except struct_error as er:
          progress("Error unpacking message ('%s', %r)" % (er,l))
          return
        for nid,ii in self.nodes.iteritems():
          val = msg[ii]
          if nid in self.pnas:
            self.pnas[nid].feedPos(val)
            if 'P' in self.DEBUG:
                _DBG(self.DEBUG['P'],"RAW nid%d pos = %g",nid,val)

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

  DEFAULT_LB = 300
  DEFAULT_UB = 2700
  def __init__(self,*arg,**kw):
    self.rawpna = pololu_ProtocolNodeAdaptor(*arg,**kw)
    self.nid = self.rawpna.nid
    self.go_slack = self.rawpna.go_slack
    self.lb = self.DEFAULT_LB
    self.ub = self.DEFAULT_UB
    self.relpos = None

  def feedPos(self,pos):
    """
    Feed a position reading from
    """
    # Update calibration range
    if pos > self.ub:
        self.ub = pos
    if pos < self.lb:
        self.lb = pos
    # Position as a fraction of calibrated range
    p = (pos - self.lb)/float(self.ub - self.lb)
    self.relpos = p

  def isCalibrated(self):
      return (self.ub != self.DEFAULT_UB) and (self.lb != self.DEFAULT_LB)

  def get_relpos(self):
    return self.relpos

  def sendCmd( self, cmd ):
    self.rawpna.p.send_cmd( self.MINISSC2_BYTE, self.nid, (cmd,) )

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
  # Scale of positions
  SCL = 36000.
  # Offset of zero position
  OFS = 18000.

  def __init__(self, node_id, typecode, pna, *argv, **kwarg):
    AbstractServoModule.__init__(self, node_id, typecode, pna, *argv, **kwarg )
    self._attr.update(
      go_slack="1R",
      set_pos="2W",
      get_pos="1R",
      is_slack="1R"
      )
    self.setDefaults()

  def setDefaults(self):
    self.pos = None
    self.goalPos = 0
    self.DEBUG=DEBUG
    self.tCal = None
    self.set(
        p_punch = 6,
        n_punch = -6,
        Kp = 100.,
        mode = 0,
        torque_rpm = 0,
        slack = True,
    )

  def set(self,**kw):
      """
      Set multiple attributes
      """
      ATTR={'slack','p_punch','n_punch','Kp','mode','torque_rpm'}
      for k,v in kw.iteritems():
          if not k in ATTR:
              raise KeyError("Cannot set '%s'" % k)
      self.__dict__.update(kw)
      if kw.get('slack',False):
          self.go_slack()

  def set_mode(self, mode):
    """
    0 - Servo Mode
    1 - Motor Mode
    2 - Continuous Rotation Mode
    """
    if type(mode)==str:
        mode = mode.upper()
        ptn = {'S':'SERVO','M':'MOTOR','C':'CONTINUOUS'}[mode[0]]
        if not ptn.startswith(mode):
            raise KeyError("Unknown mode '%s'" % mode)
        mode = 'SMC'.find(mode[0])
    elif mode < 0 or mode > 2:
        raise KeyError("Unknown mode %d" % mode)
    self.mode = mode

  def get_mode(self):
    """
    Returns the current mode
    """
    return self.mode

  def set_torque(self, rpm):
      if self.mode == 1:
          self.torque_rpm = rpm
          self.slack = False
      else:
          raise ValueError("Cannot set_torque() in mode %d" % self.mode)

  def get_torque(self):
      return self.torque_rpm

  def set_rpm(self, target):
    """
    Sets the rpm

    INPUT:
    target -- float -- rate of rotation
    """
    self.slack = False
    if self.mode == 1:
        self.torque_rpm = target
    else:
        return self._set_rpm(target)

  def _set_rpm(self, target):
    """(private)
    Sets the rpm via the pna interface

    INPUT:
    target -- float -- rate of rotation
    """
    M = float(self.MAXRPM)
    cmd = max(0,min(254,int(127+128*target/M)))
    if 'r' in self.DEBUG:
        _DBG(self.DEBUG['r'], "nid%d.set_rpm(%g) --> %g",
             self.node_id, target,cmd)
    self.pna.sendCmd(cmd)

  def isCalibrated(self):
      return self.tCal is not None

  def update(self,t):
      """
      Update function that is used at startup

      Runs calibration; then replaces itself with _line_update
      """
      # If uncalibrated --> rotate
      if not self.pna.isCalibrated():
          self.set_rpm(20)
          return
      # If newly calibrated --> Store calibration time
      if self.tCal is None:
          self.tCal = t
          return
      # For a while, pound with stop, go slack commands
      if t-self.tCal<0.5:
          self.torque_rpm = 0
          self.set_rpm(0)
          self.go_slack()
      else: # and then switch to controller
          self.update = self._live_update

  def _live_update( self, t ):
        """
        Perform a periodic update for controlling motor position
        """
        # Get current position; range is 0-1
        np = self.pna.get_relpos()
        if np is None:
            return
        self.pos = np
        # Do nothing else if slack
        if self.slack:
            return
        if self.mode == 0: # servo
            err = self.goalPos - np
            rpm = -self.Kp*err
        elif self.mode == 1: # motor mode
            rpm = self.torque_rpm
        elif self.mode == 2: # continuous
            err = self.goalPos - np
            if err>0.5:
                err -= 1
            elif err<-0.5:
                err += 1
            rpm = -self.Kp*err
        else:
            assert False,"Bad mode %d" % self.mode
        if 'u' in self.DEBUG:
            _DBG(self.DEBUG['u'],"nid%d update pos %g goal %g rpm %g mode %d",
                self.node_id, np, self.goalPos, rpm, self.mode)
        if rpm>0:
            rpm += self.p_punch
        elif rpm<0:
            rpm += self.n_punch
        return self._set_rpm(rpm)

  def set_pos( self, pos):
        """sets goal position in centi-degrees (-18000 to 18000)"""
        if self.mode == 1: # motor mode
            raise ValueError("Cannot set_pos() in mode 1 MOTOR")
        p = (int(pos) + self.OFS) % self.SCL
        self.goalPos = p/self.SCL
        self.slack = False

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
        return self.pos*self.SCL-self.OFS

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
