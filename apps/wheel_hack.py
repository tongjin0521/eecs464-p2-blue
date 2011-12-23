from joy import *
import ckbot.hitec_hack as hc
import ckbot.logical as logical
from math import copysign,cos,sin,pi


class RapidQuadApp( JoyApp ):
  """
  Simple of example of using 4 hitec CR servos to tank steer using joy stick
  """
  def __init__(self,*arg,**kw):

    # set up protocol with hitec bus
    nodes = range(0,128)
    
    b = hc.Bus('/dev/ttyUSB0')
    p = hc.Protocol(bus = b, nodes = nodes)

    kw.update(robot=dict(protocol = p))
    JoyApp.__init__(self, confPath="./cfg/JoyAppWheel.yml", *arg,**kw)
    self.r = self.robot.at

    self.amp_h= 500 #2000
    self.amp_k= 2000 #4000
    self.drive = 0
    self.steer = 0
    self.vel_gain = 250
    self.up_ang= 0
    self.low_ang = 0
    self.ofs_h = 375

  def zeros(self):
    print "Setting Motor Module Speeds to Zero"
    self.r.FL.set_speed(0)
    self.r.FR.set_speed(0)
    self.r.BL.set_speed(0)
    self.r.BR.set_speed(0)

    self.r.low.go_slack()
    self.r.up.go_slack()
    
  def fun(self,phase):
    Kd = 500     #driving gain
    Ks = 250     #steering gain

    self.r.FL.set_speed(Kd*self.drive-Ks*self.steer)
    self.r.FR.set_speed(-(Kd*self.drive+Ks*self.steer))
    self.r.BL.set_speed(Kd*self.drive-Ks*self.steer)
    self.r.BR.set_speed(-(Kd*self.drive+Ks*self.steer))
      
    self.r.low.set_pos(self.up_ang)
    self.r.up.set_pos(self.low_ang)

  def rapid_quadOnStart(self):
    progress("Quad: starting")

  def onStart(self):

    #bus supports 342[commands/sec]/[7 modules] = 48.8 updates/sec.
    #minimum interval between function calls is 1/488 = 0.02
    self.rapid_quad_plan = FunctionCyclePlan(self,self.fun,N=20,maxFreq=10)
    self.rapid_quad_plan.onStart = self.rapid_quadOnStart
    self.rapid_quad_plan.onStop = curry(progress,"Rapid_Quad: done")
    self.rapid_quad_plan.setFrequency(1.0)
    self.currplan = self.rapid_quad_plan

    self.rate = 0.05 #freq increase/decrease rate
    self.limit = 5.0

    sf = StickFilter(self)
    #arm: lowpass these
    sf.setFilter("joy1axis2")
    sf.setFilter("joy1axis3")

    #base: 
    sf.setFilter("joy0axis2")
    sf.setFilter("joy0axis3")

    sf.start()
    self.sf = sf
    self.timeToShow = self.onceEvery(0.25)


  def onStop(self):
      if self.currplan.isRunning():
        self.currplan.stop()
      print " Setting Modules to zeros "
      self.zeros()

  def onEvent(self,evt):
    self.drive=self.sf.getValue("joy0axis3")
    self.steer=self.sf.getValue("joy0axis2")
    self.low_ang+=self.vel_gain*self.sf.getValue("joy1axis2")
    self.up_ang+=self.vel_gain*self.sf.getValue("joy1axis3")

    if self.timeToShow():
      progress('drive changed to %0.2f' % self.drive)
      progress('steer changed to %0.2f' % self.steer)
      progress('low_ang changed to %0.2f' % self.low_ang)
      progress('up_ang changed to %0.2f' % self.up_ang)


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
      if self.currplan.isRunning():
        self.currplan.stop()
      self.zeros()
    elif evt.key == K_ESCAPE:
      self.stop()

if __name__=="__main__":
  robot = None
  scr = None

  app = RapidQuadApp(robot=robot,scr=scr)
  app.run()
