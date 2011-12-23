from joy import *
import ckbot.hitec as hitec
import centipede
from math import copysign,cos,sin,pi

class FlipNWormPlan( Plan ):
  def __init__(self,app,flip,worm,*arg,**kw):
    Plan.__init__(self,app,*arg,**kw)
    self.flip = flip
    self.worm = worm
    
class  FlipNWormApp( JoyApp ):

  def __init__(self,flipSpec,wormSpec,*arg,**kw):

    # set up protocol with hitec bus
    nodes = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
    bus = hitec.Bus()
    p = hitec.Protocol(bus = bus, nodes = nodes)

    kw.update(robot=dict(protocol = p))
    JoyApp.__init__(self, confPath="./cfg/JoyAppWheel.yml", *arg,**kw)
    #JoyApp.__init__(self, *arg,**kw)
    self.flipSpec = flipSpec
    self.wormSpec = wormSpec
    self.r = self.robot.at

  def fun(self,phase):

    amp = 2000
    ofs = 2*pi/4
    w = 2*pi
    leg_pos_cater = 9000

    self.r.L1.set_pos(leg_pos_cater)
    self.r.L2.set_pos(leg_pos_cater)
    self.r.L3.set_pos(leg_pos_cater)

    self.r.B1.set_pos(amp*sin(w*phase+0*ofs))
    self.r.F2.set_pos(amp*sin(w*phase+1*ofs))
    self.r.B2.set_pos(amp*sin(w*phase+2*ofs))
    self.r.F3.set_pos(amp*sin(w*phase+3*ofs))


  def caterpillarOnStart(self):
    progress("Flip: starting") 
    leg_pos_cater = 9000
    self.r.L1.set_pos(leg_pos_cater)
    self.r.L2.set_pos(leg_pos_cater)
    self.r.L3.set_pos(leg_pos_cater)


  def onStart(self):
    self.flipplan = SheetPlan(self, loadCSV("flip.csv"))
    self.flipplan.onStart = lambda : progress("Flip: starting") 
    self.flipplan.onStop = lambda : progress("Flip: done") 

    self.wormplan = SheetPlan(self, loadCSV("worm.csv"))
    self.wormplan.onStart = lambda : progress("Worm: starting") 
    self.wormplan.onStop = lambda : progress("Worm: done") 

    #bus supports 342[commands/sec]/[7 modules] = 48.8 updates/sec.
    #minimum interval between function calls is 1/488 = 0.02
    self.caterpillarplan = FunctionCyclePlan(self,self.fun,N=20,maxFreq=10)
    self.caterpillarplan.onStart = self.caterpillarOnStart
    self.caterpillarplan.onStop = curry(progress,"Caterpillar: done")

    self.caterpillarplan.setFrequency(1.5)


    self.hex = centipede.Hexapod(self.r.B1, self.r.L1,
                                 self.r.F2, self.r.B2, self.r.L2,
                                 self.r.F3, self.r.L3)
    self.hexapodplan = FunctionCyclePlan(self, self.hex.fun, N=30, maxFreq=10, interval=0)
    self.hexapodplan.onStart = curry(progress,"Hexapod: starting")
    self.hexapodplan.onStop = curry(progress,"Hexapod: done")
    self.hex.w = 1.3
    self.hex.b.rollAmp = 0.5
    self.hexapodplan.setFrequency(1)

    self.currplan = self.hexapodplan

    self.rate = 0.05 #freq increase/decrease rate
    self.limit = 5.0
    self.rollrate = 0.01
    self.rolllimit = 8500
    self.yawrate = 0.01
    self.yawlimit = 2000
    self.ecclimit = pi/2
    self.dir = 1                                                                                               
    sf = StickFilter(self)
    sf.setLowpass("joy0axis2",10)
    sf.setLowpass("joy0axis3",10)

    sf.start()
    self.sf = sf
    self.timeToShow = self.onceEvery(0.25)


  def onStop(self):
      if self.currplan.isRunning():
        self.currplan.stop()
      print " Setting Modules Slack "
      self.robot.off()

  def onEvent(self,evt):
    if self.timeToShow():
      self.hex.b.bend=-0.5*self.sf.getValue("joy0axis2")
      self.hex.b.yawAmp=-self.sf.getValue("joy0axis3")
      progress('Yaw amplitude changed to %0.2f' % self.hex.b.yawAmp)
      progress('Roll amplitude changed to %0.2f' % self.hex.b.rollAmp)
      progress('Bend changed to %0.2f' % self.hex.b.bend)
      #progress('Phase offset changed to %s' % str(self.b.ecc))
      if self.caterpillarplan.isRunning():
        progress('Frequency changed to %.2f Hz' %self.currplan.getFrequency())

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
    if evt.type != KEYDOWN:
      return
    # assertion: must be a KEYDOWN event 
    if evt.key == K_f:
      if ( not self.flipplan.isRunning()):
        self.flipplan.start()
        self.currplan = self.flipplan
    elif evt.key == K_w:
      if ( not self.wormplan.isRunning()):
        self.wormplan.start()
        self.currplan = self.wormplan
    elif evt.key == K_c:
      if ( not self.caterpillarplan.isRunning() ):
        self.caterpillarplan.start()    
        self.currplan = self.caterpillarplan
    elif evt.key == K_h:
      if ( not self.hexapodplan.isRunning() ):
        self.hexapodplan.start()    
        self.currplan = self.hexapodplan
    elif evt.key == K_u:
      if ( not self.flipplan.isRunning() 
           and not self.wormplan.isRunning()):
        print "  - Updating worm.csv and flip.csv"
        self.wormplan.update(loadCSV("worm.csv"))
        self.flipplan.update(loadCSV("flip.csv"))
    elif evt.key == K_SPACE:
      if self.currplan.isRunning():
        self.currplan.stop()
      print " Setting Modules Slack "
      self.robot.off()
    elif evt.key == K_ESCAPE:
      self.stop()


    #frequency
    elif evt.key in (ord(','),ord('.')):
      if self.hexapodplan.isRunning():
        f=self.hex.w
        if evt.key==ord(','):
          newf = (1-self.rate)*f - self.rate*self.limit
        else:
          newf = (1-self.rate)*f - self.rate*self.limit
        self.hex.w=copysign(newf,f)

      else:
        f=self.currplan.getFrequency()
      # Change frequency up/down in range -limit..limit hz
        if evt.key==ord(','):
          f = (1-self.rate)*f - self.rate*self.limit
        else:
          f = (1-self.rate)*f + self.rate*self.limit
        if abs(f) < 1.0/self.limit:
          self.currplan.setPeriod( 0 )
        else:
          self.currplan.setFrequency(copysign(newf,self.currplan.getPeriod()))
if __name__=="__main__":
  robot = None
  scr = None
  flipSpec = "#flip "
  wormSpec = "#worm "
  args = list(sys.argv[1:])
  while args:
    arg = args.pop(0)
    if arg=='--mod-count' or arg=='-c':
      N = int(args.pop(0))
      robot = dict(count=N)
    elif arg=='--flip' or arg=='-s':
      flipSpec = args.pop(0)
      if flipSpec[:1]==">": scr = {}
    elif arg=='--wormcut' or arg=='-h':
      wormSpec = args.pop(0)
      if wormSpec[:1]==">": scr = {}
    elif arg=='--help' or arg=='-h':
      sys.stdout.write("""
  Usage: %s [options]
  
    'Shave and a Haircut' example of running Plan-s in parallel and in
    sequence. The example shows that plans can run in parallel, and that
    Plan behaviors (e.g. the ShaveNHaircutPlan defined here) can use other
    plans as sub-behaviors, thereby "calling them" sequentially.
    
    When running, the demo uses the keyboard. The keys are:
      's' -- start "Shave"
      'h' -- start "Haircut"
      'escape' -- exit program
      
    Options:      
      --mod-count <number> | -c <number>
        Search for specified number of modules at startup
      
      --flip <spec> | -s <spec>
      --Wormcut <spec> | -h <spec>
        Specify the output setter to use for 'flip' (resp. 'wormcut')
        
        Typical <spec> values would be:
         '#flip ' -- to print messages to the terminal with '#flip ' as prefix
         '>x' -- send to Scratch sensor 'x'
         'Nx3C/@set_pos' -- send to position of CKBot servo module with ID 0x3C
        
        NOTE: to use robot modules you MUST also specify a -c option
        
    """ % sys.argv[0])
      sys.exit(1)
    # ENDS cmdline parsing loop
  
  app = FlipNWormApp(flipSpec,wormSpec,robot=robot,scr=scr)
  app.run()
