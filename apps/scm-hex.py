from joy.decl import *
from joy import JoyApp, FunctionCyclePlan, progress, DEBUG
from numpy import nan

SERVO_NAMES = {
   0x01: 'FL', 0x02 : 'ML', 0x03 : 'HL',
   0x04: 'FR', 0x05 : 'MR', 0x06 : 'HR'
}

class ServoWrapper( object ):
    def __init__( self, servo, **kw ):
        self.servo = servo
        self.aScl = 1.0
        self.rpmScl = 1/64.0
        self.posOfs = 0
        self.ori = 1
        self.isInDZ = None
        self.__dict__.update(kw)
        self._ensure_motor = lambda : None
        self._ensure_servo = lambda : None
        print "DEBUG = ", repr(DEBUG)
   
    def _set_servo( self ):
        """(private) set module in servo mode

            also configures the _ensure_* callbacks        
        """
        self.servo.set_mode('Servo')
        if "h" in DEBUG:
            progress("%s.set_mode('Servo')" % (self.servo.name) )
        self._ensure_servo = lambda : None
        self._ensure_motor = self._set_motor
    
    def _set_motor( self ):
        """(private) set module in motor mode

            also configures the _ensure_* callbacks        
        """
        self.servo.set_mode('Motor')
        if "h" in DEBUG:
            progress("%s.set_mode('Motor')" % (self.servo.name) )
        self._ensure_motor = lambda : None
        self._ensure_servo = self._set_servo
                    
    def set_ang( self, ang ):
        self._ensure_servo()
        pos = self.posOfs + self.ori * self.aScl * ang
        if pos < -16000:
            pos = -16000
        elif pos > 16000:
            pos = 16000
        self.isInDZ = False
        self.servo.set_pos(pos)
        if "h" in DEBUG:
            progress("%s.set_angle(%g) <-- angle %g" % (self.servo.name,pos,ang) )
    
    def get_ang( self ):
        pos = self.servo.get_pos()
        if pos > 16600 or pos < -16600:
            self.isInDZ = True
            return nan # in dead zone
        ang = (pos - self.posOfs) / self.aScl / self.ori
        self.isInDZ = False
        if "h" in DEBUG:
            progress("%s.get_angle --> %g (%s)" % (self.servo.name,ang,self.isInDZ) )
        return ang
    
    def set_rpm( self, rpm ):
        self._ensure_motor()
        tq = self.rpmScl * self.ori * rpm
        if tq<-1:
            tq = -1
        elif tq>1:
            tq = 1
        if "h" in DEBUG:
            progress("%s.set_torque(%g) <-- rpm %g" % (self.servo.name,tq, rpm) )
        self.servo.set_torque(tq)
        
    
class SCMHexApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
    
  def onStart( self ):
    DEBUG.append('h')
    self.leg = [
      ServoWrapper(self.robot.at.FL, ori = -1),
      ServoWrapper(self.robot.at.ML, ori = -1, tqScl = 0.65/64),
      ServoWrapper(self.robot.at.HL, ori = -1),
      ServoWrapper(self.robot.at.FR),
      ServoWrapper(self.robot.at.MR, tqScl = 0.65/64),
      ServoWrapper(self.robot.at.HR),
    ]
    self.triL = self.leg[ 0::2 ]
    self.triR = self.leg[ 1::2 ]
    for leg in self.triL:
        leg.set_ang(9000)
    for leg in self.triR:
        leg.set_ang(-9000)
    self.fcp = FunctionCyclePlan(self, self._fcp_fun, 24)
    self.freq = 1.0
    self.rate = 0.05
    self.limit = 1/0.45

  def _leg_to_phase( self, leg, phase ):
      """
      Move a leg to a given phase, in position mode      
      """
      # if away from dead-zone phase
      if abs(phase-0.5)>0.1:
          # If leg might still be in dead zone --> poll it
          if leg.isInDZ:
              leg.get_pos()
          # If outside dead zone --> use position control
          if not leg.isInDZ:
              leg.set_ang( (phase-0.5)*36000 )
      else: # else --> phase in dead zone range; switch to RPM control
          # If not moving --> abort this
          if self.fcp.period == 0:
              return
          rpm = 60 / self.fcp.period
          leg.set_rpm( rpm )
          
  def _fcp_fun( self, phase ):
      progress('fun')
      for leg in self.triL:
            self._leg_to_phase( leg, phase )
      phase = (phase + 0.5) % 1.0
      for leg in self.triR:
            self._leg_to_phase( leg, phase )
    
      
  def onEvent(self, evt):
    if evt.type==KEYDOWN:
      if evt.key in [ord('q'),27]: # 'q' and [esc] stop program
        self.stop()
        #
      elif evt.key==K_SPACE: # [space] stops cycles
        self.fcp.setPeriod(0)
        progress('Period changed to %s' % str(self.fcp.period))
        #
      elif evt.key in (K_COMMA, K_PERIOD):
        f = self.freq
        # Change frequency up/down in range -limit..limit hz
        if evt.key==K_COMMA:
          f = (1-self.rate)*f - self.rate*self.limit
        else:
          f = (1-self.rate)*f + self.rate*self.limit
        if abs(f) < 1.0/self.limit:
          self.fcp.setPeriod( 0 )
        else:
          self.fcp.setPeriod( 1/f )
        self.freq = f
        progress('Period changed to %g, %.2f Hz' % (self.fcp.period,f))
        #
      elif evt.key==K_h:
        progress( "HELP: ',' to decrease/reverse period; '.' opposite; ' ' stop; 'q' end program; 'h' this help; other keys start Plan" )
        #
      else: # any other key
        progress( "Starting cycles...." )
        self.fcp.start()
      return # (from all keypress handlers)
    if evt.type not in [TIMEREVENT, JOYAXISMOTION, MOUSEMOTION]:
      JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  print """
  SCM Hexapod controller
  ----------------------
  
  When any key is pressed, starts a FunctionCyclePlan that runs the SCM
  hexapod. Usually works in position mode, but crosses the dead zone by
  switching between position and speed control, giving approximate RPM
  command, and polling for the motor to be out the other side
  
  h -- help
  q -- quit
  , and . -- change rate
  SPACE -- pause / resume
  any other key -- start 
  
  The application can be terminated with 'q' or [esc]
  """

  import ckbot.dynamixel as DX
  import ckbot.logical as L
  import ckbot.nobus as NB
  import joy

  if 0: # 1 to run code; 0 for simulation
      # for actual operation use w/ arch=DX & NO required line:  
      L.DEFAULT_BUS = DX
      L.DEFAULT_PORT = dict(
          TYPE="tty", baudrate=57600, glob="/dev/ttyUSB*"
      )
      app=SCMHexApp(
          robot=dict(arch=DX,count=len(SERVO_NAMES), names=SERVO_NAMES)
      )
  else:
      L.DEFAULT_BUS = NB
      app=SCMHexApp(
          robot=dict(
              arch=NB,count=len(SERVO_NAMES),fillMissing=True,
              required=SERVO_NAMES.keys(), names=SERVO_NAMES
      ))

  joy.DEBUG[:]=[]
  app.run()

