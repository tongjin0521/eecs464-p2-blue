from joy import *
import ckbot.hitec as hitec
import ckbot.logical as logical
from math import copysign,cos,sin,pi
import time

class SnakeApp( JoyApp ):
  """
  Simple of example of using 4 hitec CR servos to tank steer using joy stick
  """
  def __init__(self,*arg,**kw):
    self.bus = hitec.Bus()
    # set up protocol with hitec bus    
    p = hitec.Protocol(bus = self.bus)

    kw.update(robot=dict(protocol = p, count=5))
    JoyApp.__init__(self, confPath="./cfg/JoyAppSnake.yml", *arg,**kw)
    self.r = self.robot.at

    self.amp_h= 2000
    self.amp_v= 2000

  def zeros(self):
    print "Setting modules to Zero"
    self.r.H1.go_slack()
    self.r.H2.go_slack()
    self.r.H3.go_slack()
    self.r.H4.go_slack()
    self.r.V1.go_slack()


  def funSnake(self,phase):
    self.r.H1.set_pos(self.amp_h*sin(2*pi*phase))
    self.r.H2.set_pos(0.75*self.amp_h*sin(2*pi*phase + 0.25*pi))
    self.r.H3.set_pos(0.75*self.amp_h*sin(2*pi*phase + 0.5*pi))
    self.r.H4.set_pos(self.amp_h*sin(2*pi*phase + 0.75*pi))

    self.r.V1.set_pos(self.amp_v*self.steer)

  
  def onStart(self):
    self.r.H1.zero_ofs=-2000
    self.r.H2.zero_ofs=0
    self.r.H3.zero_ofs=1000
    self.r.H4.zero_ofs=-1000
    self.r.V1.zero_ofs=-1000

    #bus supports 342[commands/sec]/[7 modules] = 48.8 updates/sec.
    #minimum interval between function calls is 1/488 = 0.02
    snakePlan = FunctionCyclePlan(self,self.funSnake,N=40,maxFreq=10,interval=0.01)
    snakePlan.onStart = curry(progress,"Starting")
    snakePlan.onStop = curry(progress,"Stopping")
    snakePlan.setFrequency(1.0)
    
    self.snakePlan = snakePlan
    self.currplan = self.snakePlan
    self.rate = 0.05 #freq increase/decrease rate
    self.limit = 5.0

    sf = StickFilter(self)
    #arm:
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
    self.drive= 0
    self.steer=self.sf.getValue("joy0axis2")
    self.drive = 0

    f = 2*self.sf.getValue("joy0axis3")
    self.currplan.setFrequency(f)

    #arm
    #self.low_ang=self.sf.getValue("joy0axis2")
    #self.up_ang=self.sf.getValue("joy0axis3")
    
    if self.timeToShow():
      progress('drive %5.2f steer %5.2f rate %5.2f'
        % (self.amp_h,self.steer,self.currplan.getFrequency()))

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
    if evt.key == K_s: 
      if ( not self.snakePlan.isRunning() ):
        self.snakePlan.start()
        self.currplan = self.snakePlan
    elif evt.key == K_SPACE:
      if self.currplan.isRunning():
        self.currplan.stop()
      self.zeros()
    elif evt.key in [K_ESCAPE,K_q]:
      self.stop()

    #frequency
    elif evt.key in (ord(','),ord('.')):
      f=self.currplan.getFrequency()
      # Change frequency up/down in range -limit..limit hz
      if evt.key==ord(','):
        f = (1-self.rate)*f - self.rate*self.limit
      else:
        f = (1-self.rate)*f + self.rate*self.limit
      if abs(f) < 1.0/self.limit:
        self.currplan.setPeriod( 0 )
      else:
        self.currplan.setFrequency(copysign(f,self.currplan.getPeriod()))

if __name__=="__main__":
  robot = None
  scr = None

  app = SnakeApp(robot=robot,scr=scr)
  app.run()
