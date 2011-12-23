from joy import *
import ckbot.hitec as hitec
import ckbot.logical as logical
from math import copysign,cos,sin,pi


class RapidQuadApp( JoyApp ):
  """
  Simple of example of using 4 hitec CR servos to tank steer using joy stick
  """
  def __init__(self,*arg,**kw):
    self.bus = hitec.Bus()
    # set up protocol with hitec bus    
    p = hitec.Protocol(bus = self.bus)

    kw.update(robot=dict(protocol = p, count=4))
    JoyApp.__init__(self, confPath="$/cfg/JoyAppWheel.yml", *arg,**kw)
    self.r = self.robot.at

    self.amp_h= 500 #2000
    self.amp_k= 2000 #4000
    self.drive = 0
    self.steer = 0
    self.up_ang= 0
    self.low_ang = 0
    self.ofs_h = 375

  def zeros(self):
    print "Setting Motor Module Speeds to Zero"
    self.r.FL.set_speed(0)
    self.r.FR.set_speed(0)
    self.r.BL.set_speed(0)
    self.r.BR.set_speed(0)

  def fun(self,phase):
    Kd = 500     #driving gain
    Ks = 250     #steering gain
    self.r.FL.set_speed(Kd*self.drive-Ks*self.steer)
    self.r.FR.set_speed(-(Kd*self.drive+Ks*self.steer))
    self.r.BL.set_speed(Kd*self.drive-Ks*self.steer)
    self.r.BR.set_speed(-(Kd*self.drive+Ks*self.steer))
    
  def onStart(self):

    #bus supports 342[commands/sec]/[7 modules] = 48.8 updates/sec.
    #minimum interval between function calls is 1/488 = 0.02
    rqp = FunctionCyclePlan(self,self.fun,N=20,maxFreq=10,interval=0.1)
    rqp.onStart = curry(progress,"Starting")
    rqp.onStop = curry(progress,"Stopping")
    rqp.setFrequency(1.0)
    
    self.rapid_quad_plan = rqp
    self.currplan = self.rapid_quad_plan

    self.rate = 0.05 #freq increase/decrease rate
    self.limit = 5.0

    sf = StickFilter(self)
    #base:
    sf.setLowpass("joy0axis2",5)
    sf.setLowpass("joy0axis3",5)

    sf.start()
    self.sf = sf
    self.timeToShow = self.onceEvery(0.25)
    self.timeToSend = self.onceEvery(0.05)
    self.t0 = 0

  def onStop(self):
      if self.currplan.isRunning():
        self.currplan.stop()
      print " Setting Modules to zeros "
      self.zeros()

  def onEvent(self,evt):
    self.drive=self.sf.getValue("joy0axis3")
    self.steer=self.sf.getValue("joy0axis2")
    
    if self.timeToShow():
      progress('drive %5.2f steer %5.2f'
        % (self.drive,self.steer))

    if evt.type==JOYBUTTONDOWN and evt.joy==0:
      progress( describeEvt(evt) )
      # start
      if evt.button==5:
        self.currplan.start()
        progress('(say) starting plan')
      # pause/play
      if evt.button==7:
        self.currplan.stop()
        progress("(say) setting Modules Slack ")
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
    elif evt.key in [K_ESCAPE,K_q]:
      self.stop()
    elif evt.key == K_UP:
      self.drive += 0.1
    elif evt.key == K_DOWN:
      self.drive -= 0.1

if __name__=="__main__":
  robot = None
  scr = None

  app = RapidQuadApp(robot=robot,scr=scr)
  app.run()
