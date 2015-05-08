from joy import *
from time import time as now
import ckbot.dynamixel as DX
import ckbot.logical as logical
import ckbot.nobus as NB

from centipedeContact import gaitParams, contactGait

class Segment(object):
  def __init__(self, front, back, leg):
    self.f = front
    self.b = back
    self.l = leg
        
  def set_pos(self,yaw,bend,roll):
    A = matrix([[2,0.5,0],[-2,0.5,0],[0,0,1]],dtype='float')
    pos=A*matrix([[yaw],[bend],[roll]])
    if self.f!=None:
      self.f.set_pos(pos[0])
    if self.b!=None:
      self.b.set_pos(pos[1])
    if self.l!=None:
      self.l.set_pos(pos[2])

class MechapodApp( JoyApp ):

  def __init__(self,*arg,**kw):
    assert not kw.has_key('cfg')
    cfg = dict(
       maxFreq = 3,
       freq = 1
     )
    JoyApp.__init__(self, confPath="$/cfg/mechapod.yml", cfg=cfg, *arg,**kw)
    self.stickMode = False    
    self.last = now()

  def _tripodGaitFun( self, phi ):  		      
    g1 = self.gait1
    g2 = self.gait2
    g3 = self.gait3
    
    #for legs 1 & 3
    g1.manageGait(phi)
    s1roll = degrees(g1.roll)
    s1yaw = degrees(g1.yaw)
    
    g3.manageGait(phi)
    s3roll = degrees(g3.roll)
    s3yaw = degrees(g3.yaw)
	
    #for leg 2: give phi2 = (phi1 + 0.5) mod 1
    if(phi > 0.5):
        phi2 = phi -0.5
    else:
        phi2 = phi + 0.5
        
    g2.manageGait(phi2) 
    s2roll = degrees(g2.roll)
    s2yaw = degrees(g2.yaw)
            
    #roll/yaw are deg, need to convert to centidegrees
    self.S1.set_pos(s1yaw*100, g1.bend, s1roll*100)  
    self.S2.set_pos(s2yaw*100, g2.bend, s2roll*100)   
    self.S3.set_pos(s3yaw*100, g3.bend, s3roll*100)      #facing opposite     


  def onStart(self):
    self.S1 = Segment(None, self.robot.at.B1, self.robot.at.L1)
    self.S2 = Segment(self.robot.at.F2, self.robot.at.B2, self.robot.at.L2)
    self.S3 = Segment(self.robot.at.F3, None, self.robot.at.L3)
    #set the slope parameter for each motor:
    for m in self.robot.itermodules():
      if isinstance( m, DX.MX64Module ):
        continue
      m.mem[m.mcu.ccw_compliance_slope] = 64
      m.mem[m.mcu.cw_compliance_slope] = 64

    self.last = 0
    self.stickMode = False
    
    #setup parameters for contact gait - reasonable vals for hexapod demo
    gp = gaitParams()
    gp.rollThresh = -(25*pi/180)
    gp.yawThresh = -(8*pi/180)
    gp.maxRoll = (26*pi/180)
    gp.rollAmp = (26*pi/180)
    gp.yawAmp = (9*pi/180)
    gp.stanceVel = 1.26
    
    self.gait1 = contactGait(gp, self.S1.l)
    self.gait2 = contactGait(gp, self.S2.l)
    self.gait3 = contactGait(gp, self.S3.l)

    self.tripodGait = FunctionCyclePlan(self, self._tripodGaitFun, N=302,interval=0.02)
    self.tripodGait.setFrequency(self.cfg.freq)
    
    sf = StickFilter(self)
    sf.setLowpass("joy0axis0",10)
    sf.setLowpass("joy0axis1",10)
    sf.start()
    self.sf = sf
    
    self.timeToShow = self.onceEvery(0.25)
    self.timeToUpdate = self.onceEvery(0.05)

  def onStop(self):
    self.robot.off()
  
  def onEvent(self, evt):
    tg = self.tripodGait
    # If it's time, display status message
    if self.timeToShow():
      progress('freq: %g, bend: %g, stickMode: %g' 
               % (tg.getFreqency(), self.gait1.bend, self.stickMode))
    # If it's time, process stick mode commands and send into robot
    if self.timeToUpdate():
      if self.stickMode:
        #update gait with joystick inputs
        bend = self.sf.getValue("joy0axis0") #value range: [-1, 1]
        freq = self.sf.getValue("joy0axis1") #value range: [-1, 1]
        tg.setFrequency(freq * self.cfg.maxFreq)
        b = self.cfg.maxBend * bend
        self.gait1.bend = b 
	self.gait2.bend = b
        self.gait3.bend = b
      else:
        #return to button control
        self.plan.setFrequency(self.cfg.freq)  
        self.gait1.bend = 0
        self.gait2.bend = 0
        self.gait3.bend = 0
    # Ignore any other timer events
    if evt.type==TIMEREVENT:
	return
    if evt.type == JOYAXISMOTION:
      # Make sure StickFilter gets all JOYAXISMOTION events
      self.sf.push(evt)
      return
    # 
    #  Process other user control commands 
    #
    if ((evt.type==KEYDOWN and evt.key==K_m)
	or (evt.type==JOYBUTTONDOWN and evt.button==5)): # r2 button
          tg.start()#
	  progress("(say) Starting tripod gait")
    elif ((evt.type==KEYDOWN and evt.key==K_s)
	or (evt.type==JOYBUTTONDOWN and evt.button==7)): # r1 button
          tg.stop()
	  progress("(say) Stopping tripod gait")
	  tg.setFrequency(0)
    elif (evt.type==JOYBUTTONDOWN and evt.button==0):
        self.stickMode = not self.stickMode   
    # Punt to superclass
    return JoyApp.onEvent(self,evt)


if __name__=="__main__":
  print """
  Centipede
  -------------------------------
   
  When any key is pressed, a 7-module centipede commences locomotion.

  The application can be terminated with 'q' or [esc]
  """
  # for actual operation use w/ arch=DX & NO required line:  
  ckbot.logical.DEFAULT_BUS = ckbot.dynamixel
  ckbot.logical.DEFAULT_PORT = dict(TYPE="tty", baudrate=57600, glob="/dev/ttyUSB*")
  
  # for simulation use w/ arch=NB & required line:
  #ckbot.logical.DEFAULT_BUS = ckbot.nobus
  
  app=MechapodApp(robot=dict(
    arch=DX,count=7,fillMissing=True,
    #required=[ 0x26, 0x08, 0x06, 0x2D, 0x31, 0x27, 0x22 ] 
    ))
  
  app.run()

