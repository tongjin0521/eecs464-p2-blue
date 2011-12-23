from joy import *
from math import copysign,cos,sin,pi
from numpy import matrix
from numpy import linalg
import time
import ckbot.hitec

"""
WARNING

I refactored this to work with the flipnworm.py app.  FunctionCyclePlanApp needs to be fixed.

"""
class Buehler(object):
  def __init__(self): 
    "implement Buehler clock from time in range 0..1 to phase range -pi..pi wrapping twice"
    #Buehler constants
    self.duty = 0.2 # duty cycle of stance (must be 0 < duty < 0.5)
    self.ofs = pi/2 # "angle" at onset of stance
    self.sweep = 0.1 * pi # sweep range for stance (must be 0 < sweep < pi)

    
    self.rollAmp = 1.0
    self.yawAmp = 0
    self.bend = 0
    self.ecc = 0 #pi/6

    self.rollAmpDegrees = 7000
    self.yawAmpDegrees = 1000

  def buehler( self, t ):
    """
    Implements Buehler clock and returns phase as a function of time.

    INPUTS:
      t -- time -- in seconds.
    OUTPUT:
      res -- phase -- from -pi to pi
   """
    
    t -= floor(t) # wrap at 1
    if t>0.5:
      flag = 1
      t -= 0.5
    else:
      flag = 0
    if t<self.duty:
      res = self.sweep * (t / self.duty) + flag*pi + self.ofs
    else:
      res = (t-self.duty)*(pi-self.sweep)/(0.5-self.duty)+self.sweep + flag*pi + self.ofs
    res -= 2*pi*floor(res / (2*pi)) + pi
    return res

  def gait( self, t ):
    """
    INPUT:
     t -- time in seconds
    OUTPUT:
     (yaw, bend, roll) -- tuple of approximate angular position of foot in body coordinates.

    """
  #  ang = self.buehler(t)
    ang = 2*pi*t - pi;
    roll = cos(ang) * self.rollAmp*self.rollAmpDegrees
    yaw = sin(ang+self.ecc)*self.yawAmp*self.yawAmpDegrees
    bend = self.bend
    return yaw, bend, roll
 

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



class Hexapod(object) :
  """

  """
  
  def __init__(self,B1, L1, F2, B2, L2, F3, L3):
    self.S1 = Segment(None, B1, L1)
    self.S2 = Segment(F2, B2, L2)
    self.S3 = Segment(F3, None, L3)
    self.b = Buehler()
    self.w = 1.3
    self.last = 0

  def fun( self, phase ):
    ph = self.w*time.time()
    (y,b,r) = self.b.gait(ph)
    #    self.S1.set_pos(*self.b.gait(phase))
    self.S1.set_pos(y,b,-r)
    self.S2.set_pos(-y,b,r)
    self.S3.set_pos(y,b,-r)


"""
class FunctionCyclePlanApp( JoyApp ):
  def __init__(self,*arg,**kw):
#    JoyApp.__init__(self,robot=dict(count=7,timeout=10,bus=ckbot.xbee.Bus()),*arg,**kw)
#    JoyApp.__init__(self,robot=dict(count=7, timeout = 10,*arg,**kw)
    
    nodes = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
    bus = ckbot.hitec.Bus('/dev/tty.usbserial-A600e1fL')
    p = ckbot.hitec.Protocol(bus = bus, nodes = nodes)
    kw.update(robot=dict(protocol = p)) #note for uriah

    JoyApp.__init__(self, *arg,**kw)

    r = self.robot.at

    r.L1 = r.Nx02
    r.B1 = r.Nx04
    r.F2 = r.Nx05
    r.L2 = r.Nx07
    r.B2 = r.Nx03
    r.F3 = r.Nx01
    r.L3 = r.Nx06

  def onStart( self ):
    self.plan = FunctionCyclePlan(self, self.fun,90)
    self.plan.onStart = curry(progress,">>> START")
    self.plan.onStop = curry(progress,">>> STOP")
    self.plan.setFrequency(1.0)
    self.rate = 0.05
    self.limit = 5.0
    self.rollrate = 0.01
    self.rolllimit = 8500
    self.yawrate = 0.01
    self.yawlimit = 2000
    self.ecclimit = pi/2
    self.dir = 1
#
    sf = StickFilter(self)
    sf.setIntegrator("joy0axis0",100,lower=-self.yawlimit,upper=self.yawlimit)
    sf.setIntegrator("joy0axis1",100,lower=-self.rolllimit,upper=self.rolllimit)
   #sf.setIntegrator("joy0axis2",100,lower=-(self.yawlimit-self.b.maxYaw),upper=(self.yawlimit-self.b.maxYaw))
    sf.setIntegrator("joy0axis2",0.1,lower=-self.ecclimit,upper=self.ecclimit)
    sf.setIntegrator("joy0axis3",0.1,lower=0,upper=self.limit)
    sf.start()
    self.sf = sf
    self.timeToShow = self.onceEvery(0.25)
    
  def onStop(self):
    self.robot.off()
  
  def onEvent(self, evt):
    
    if self.timeToShow():
      #Every 0.25 s, yaw, roll, and frequency are updated based on the values input via joystick
      
      # self.b.maxYaw=self.sf.getValue("joy0axis0")
      self.b.maxYaw=500
      progress('Yaw amplitude changed to %s' % str(self.b.maxYaw))
      #self.b.maxRoll=-self.sf.getValue("joy0axis1")
      self.b.maxRoll=1000
      progress('Roll amplitude changed to %s' % str(self.b.maxRoll))
    #  self.b.maxBend=self.sf.getValue("joy0axis2")
    #  progress('Bend amount changed to %s' % str(self.b.maxBend))
     # self.b.ecc=self.sf.getValue("joy0axis2")
      self.b.ecc=0 #pi/2
      progress('Phase offset changed to %s' % str(self.b.ecc))
     # self.plan.setFrequency(self.dir*self.sf.getValue("joy0axis3"))
      self.plan.setFrequency(1.0)
      progress('Period changed to %g, %.2f Hz' % (self.plan.getPeriod(),self.plan.getFrequency()))
    if evt.type==JOYBUTTONDOWN and evt.joy==0:
      progress( describeEvt(evt) )
      # start
      if evt.button==5:
        self.plan.start()
        progress('--> starting plan')
      # pause/play
      if evt.button==6:
        self.stop()
      # Reverse direction
      if evt.button==7:
        self.dir=-self.dir
        progress('Direction reversed')
    if evt.type in [JOYAXISMOTION]:
      self.sf.push(evt)
      return
    if evt.type==KEYDOWN:
      if evt.key in [ord('q'),27]: # 'q' and [esc] stop program
        self.stop()
        #
      elif evt.key==ord('d'): # 'd' reverses direction
        self.dir=-self.dir
        progress('Direction reversed')
        #
      elif evt.key==ord(' '): # [space] stops cycles
        self.plan.setPeriod(0)
        self.robot.off()
        progress('Period changed to %s' % str(self.plan.period))
        #
      elif evt.key in (ord('o'),ord('p')):
        # Change yaw in range -yawlimit..yawlimit
        yaw=self.b.maxYaw
        if evt.key==ord('o'):
          yaw = (1-self.yawrate)*yaw - self.yawrate*self.yawlimit
        else:
          yaw = (1-self.yawrate)*yaw + self.yawrate*self.yawlimit
        self.b.maxYaw=yaw
        progress('Yaw amplitude changed to %s' % str(self.b.maxYaw))
        #
      elif evt.key in (ord('['),ord(']')):
        # Change roll in range -rolllimit..rolllimit
        roll=self.b.maxRoll
        if evt.key==ord('['):
          roll = (1-self.rollrate)*roll - self.rollrate*self.rolllimit
        else:
          roll = (1-self.rollrate)*roll + self.rollrate*self.rolllimit
        self.b.maxRoll=roll
        progress('Roll amplitude changed to %s' % str(self.b.maxRoll))
        #
      elif evt.key in (ord(','),ord('.')):
        f=self.plan.getFrequency()
        # Change frequency up/down in range -limit..limit hz
        if evt.key==ord(','):
          f = (1-self.rate)*f - self.rate*self.limit
        else:
          f = (1-self.rate)*f + self.rate*self.limit
        if abs(f) < 1.0/self.limit:
          self.plan.setPeriod( 0 )
        else:
          self.plan.setFrequency(copysign(f,self.plan.getPeriod()))
        self.freq = f
        progress('Period changed to %g, %.2f Hz' % (self.plan.getPeriod(),f))
        #      
      elif evt.key==ord('h'):
        progress( "HELP: ',' to decrease/reverse period; '.' opposite; ' ' stop; 'q' end program; 'h' this help; other keys start Plan" )
        #
      else: # any other key
        progress( "Starting cycles...." )
        self.plan.start()
    if evt.type!=TIMEREVENT:
      JoyApp.onEvent(self,evt)
"""



if __name__=="__main__":
  print """
  Centipede
  -------------------------------
  
  When any key is pressed, a 7-module centipede commences locomotion.

  The application can be terminated with 'q' or [esc]
  """

"""
  import joy
  joy.DEBUG[:]=[]
  app=FunctionCyclePlanApp()
  app.run()

"""
