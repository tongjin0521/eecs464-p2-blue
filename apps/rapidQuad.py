from joy import *
import ckbot.hitec as hitec
from math import copysign,cos,sin,pi


class Buehler(object):

  def __init__(self, offset, sweep, duty, period):
    """
    Parameters:
      offset (angle)
      sweep  (angle)
      duty   (ratio from 0 to 1)
      period (time in seconds)
    """
    self.offset = offset
    self.sweep = sweep
    self.duty = duty
    self.tc = period

  def step(self,t):
    """
    Implements Buehler clock and returns phase as a function of time.
      INPUTS:                                       
        t -- time -- in seconds
      OUTPUT:                                                                              
        res -- phase -- from -pi to pi                                                     
    """
    t -= floor(t) # wrap at 1
    if t<self.duty:
      res = t * (self.sweep / (self.duty*self.tc)) - self.offset
    else:
      res = (t-self.duty*self.tc)*(2*pi-self.sweep)/(self.tc-self.duty*self.tc) + self.sweep - self.offset
    res -= 2*pi*floor(res / (2*pi)) + pi
    return res

class Trot():

  def __init__(self):
    self.amp_h = 750
    self.amp_k = 1500
    self.offsets = [1700,7200,1700,7200,1700,7200,1700,7200]
#    self.offsets = [1700,6600,1800,7300,5000,7200,1700,7200]
    self.calib = [1000,5500,1000,7000,2500,7500,1200,7000]
    #    self.calib = [-700,-1100,-800,-300,-2500,300,-1500,200]
    self.ofs = 2.0*pi/2.0 #offset between phase_1 and phase_2
    self.h_ofs = 375
    self.k_ofs = pi
    self.w = 1

  def traj(self, t):
    b1 = Buehler(pi/2, pi, 0.6, self.w)
    b2 = Buehler(3*pi/2, pi, 0.6, self.w)
    ph1 = b1.step(t)
    ph2 = b2.step(t)
    h11 = -(self.calib[0]+self.offsets[0]-self.amp_h*cos(ph2 + 0.0*self.ofs)-self.h_ofs)
    k11 = self.calib[1]+self.offsets[1]-max(self.amp_k*sin(ph2 + 0.0*self.ofs + self.k_ofs),0.0)
    h12 = -(self.calib[2]+self.offsets[2]-self.amp_h*cos(ph1 + 0.0*self.ofs)-self.h_ofs)
    k12 = self.calib[3]+self.offsets[3]-max(self.amp_k*sin(ph1 + 0.0*self.ofs + self.k_ofs),0.0)
    h21 = -(self.calib[4]+self.offsets[4]-self.amp_h*cos(ph1 + 0.0*self.ofs)-self.h_ofs)
    k21 = self.calib[5]+self.offsets[5]-max(self.amp_k*sin(ph1 + 0.0*self.ofs + self.k_ofs),0.0)
    h22 = -(self.calib[6]+self.offsets[6]-self.amp_h*cos(ph2 + 0.0*self.ofs)-self.h_ofs)
    k22 = self.calib[7]+self.offsets[7]-max(self.amp_k*sin(ph2 + 0.0*self.ofs + self.k_ofs),0.0)
    return (h11, k11, h12, k12, h21, k21, h22, k22)

"""
class Hop():

  def __init__(self):
    self.amp_hop = 4000
    self.w = 1

  def traj(self,t)
    b = Buehler(pi/6, pi, 0.6, self.w)
    phase = b.step(t)
    return ( , )
"""

class Walk():
  def __init__(self):
    self.calib =[-700,-1200,-800,-300,-2500,300,-1500,200]
    self.offsets = [1700,7200,1700,7200,1700,7200,1700,7200]
    self.amp_h = 750
    self.amp_k = 1500
    self.ofs = 2.0*pi/2.0 #offset between phase_1 and phase_2
    self.h_ofs = 375#0
    self.k_ofs = pi
    self.w = 1

  def traj(self,t):
    b1 = Buehler(pi/2, pi, 0.6, self.w)
    b2 = Buehler(2*pi/2, pi, 0.6, self.w)
    b3 = Buehler(3*pi/2, pi, 0.6, self.w)
    b4 = Buehler(4*pi/2, pi, 0.6, self.w)
    ph1 = b1.step(t)
    ph2 = b2.step(t)
    ph3 = b3.step(t)
    ph4 = b4.step(t)
    h11 = -(self.calib[0]+self.offsets[0]-self.amp_h*cos(ph2 + 0.0*self.ofs)-self.h_ofs)
    k11 = self.calib[1]+self.offsets[1]-max(self.amp_k*sin(ph2 + 0.0*self.ofs + self.k_ofs),0.0)
    h12 = -(self.calib[2]+self.offsets[2]-self.amp_h*cos(ph4 + 0.0*self.ofs)-self.h_ofs)
    k12 = self.calib[3]+self.offsets[3]-max(self.amp_k*sin(ph4 + 0.0*self.ofs + self.k_ofs),0.0)
    h21 = -(self.calib[4]+self.offsets[4]+self.amp_h*cos(ph1 + 0.0*self.ofs)-self.h_ofs)
    k21 = self.calib[5]+self.offsets[5]-max(self.amp_k*sin(ph1 + 0.0*self.ofs + self.k_ofs),0.0)
    h22 = -(self.calib[6]+self.offsets[6]-self.amp_h*cos(ph3 + 0.0*self.ofs)-self.h_ofs)
    k22 = self.calib[7]+self.offsets[7]-max(self.amp_k*sin(ph3 + 0.0*self.ofs + self.k_ofs),0.0)
    return (h11, k11, h12, k12, h21, k21, h22, k22)

class Stand():

  def __init__(self):
    self.calib =[-700,-1200,-800,-300,-2500,300,-1500,200]
    self.offsets = [1700,7200,1700,7200,1700,7200,1700,7200]
    self.w = 1

  def traj(self,t):
    self.stand = [self.calib[0]+self.offsets[0],self.calib[1]+self.offsets[1],self.calib[2]+self.offsets[2],self.calib[3]+self.offsets[3],self.calib[4]+self.offsets[4],self.calib[5]+self.offsets[5],self.calib[6]+self.offsets[6],self.calib[7]+self.offsets[7]]
    return self.stand


class Zeros():

  def __init__(self):
    self.calib =[-700,-1200,-800,-300,-2500,300,-1500,200]
    self.w = 1

  def traj(self,t):
    return self.calib



class RapidQuadApp(JoyApp):

  def __init__(self,*arg,**kw):
    bus = hitec.Bus()
    p = hitec.Protocol(bus = bus)

    kw.update(robot=dict(protocol = p, count=8))
    JoyApp.__init__(self, confPath="$/cfg/JoyAppRapidQuad.yml",*arg,**kw)
    self.r = self.robot.at

  def fun(self,phase):
    (h11, k11, h12, k12, h21, k21, h22, k22) = self.gait.traj(app.now)
    self.r.H11.set_pos(h11)
    self.r.K11.set_pos(k11)
    self.r.H12.set_pos(h12)
    self.r.K12.set_pos(k12)
    self.r.H21.set_pos(h21)
    self.r.K21.set_pos(k21)
    self.r.H22.set_pos(h22)
    self.r.K22.set_pos(k22)

  def onStart(self):

    #bus supports 342[commands/sec]/[7 modules] = 48.8 updates/sec.
    #minimum interval between function calls is 1/488 = 0.02
    self.rapid_quad_plan = FunctionCyclePlan(self,self.fun,N=10,maxFreq=10)
    self.rapid_quad_plan.onStart = curry(progress,"Rapid_Quad: starting")
    self.rapid_quad_plan.onStop = curry(progress,"Rapid_Quad: done")
    self.rapid_quad_plan.setFrequency(1.0)
    self.currplan = self.rapid_quad_plan

    self.rate = 0.05 #freq increase/decrease rate
    self.limit = 5.0
    self.dir = 1                                                                                               
    sf = StickFilter(self)
    sf.setLowpass("joy0axis2",10)
    sf.setLowpass("joy0axis3",10)

    sf.start()
    self.sf = sf
    self.timeToShow = self.onceEvery(0.25)

    self.gait = Walk()

  def onStop(self):
      if self.currplan.isRunning():
        self.currplan.stop()

  def onEvent(self,evt):
    if self.timeToShow():
      #self.amp_h=self.sf.getValue("joy0axis3")
      #J Hardcode for now
      #self.amp_h=2000#self.sf.getValue("joy0axis3")
      #progress('amplitude changed to %0.2f' % self.amp_h)
      #progress('Phase offset changed to %s' % str(self.b.ecc))
      if self.rapid_quad_plan.isRunning():
        progress('Frequency changed to %.2f Hz' %self.gait.w)

    if evt.type==JOYBUTTONDOWN and evt.joy==0:
      progress( describeEvt(evt) )
      # start
      if evt.button==5:
        self.currplan.start()
        progress('--> starting plan')
      # pause/play
      if evt.button==6:
        self.currplan.stop()
        print " Setting Modules Slack "
        self.robot.off()
      # Reverse direction
      if evt.button==7:
        self.dir=-self.dir
        progress('Direction reversed')
    if evt.type in [JOYAXISMOTION]:
      self.sf.push(evt)
      return

    # assertion: must be a KEYDOWN event    
    if evt.type != KEYDOWN:
      return  
    if evt.key == K_r:
      if ( not self.rapid_quad_plan.isRunning() ):
        self.rapid_quad_plan.start()    
        self.currplan = self.rapid_quad_plan

    elif evt.key == K_SPACE:
#      if self.currplan.isRunning():
#        self.currplan.stop()
#      print " Setting Modules to zero "
      self.gait = Zeros()
    elif evt.key == K_ESCAPE:
      self.stop()

    #frequency
    elif evt.key in (ord(','),ord('.')):
      if self.rapid_quad_plan.isRunning():
        f=self.w
        if evt.key==ord(','):
          newf = (1-self.rate)*f - self.rate*self.limit
        else:
          newf = (1-self.rate)*f + self.rate*self.limit
        self.gait.w=copysign(newf,f)

if __name__=="__main__":
  robot = None
  app = RapidQuadApp(robot=robot)
  app.run()
