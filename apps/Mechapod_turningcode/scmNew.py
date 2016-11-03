from joy.decl import *
from joy import JoyApp, FunctionCyclePlan, progress, DEBUG, Plan
from numpy import nan, asfarray, prod, isnan, pi, clip, sin, angle, sign, round
from cmath import exp
from time import sleep, time as now
from ckbot.dynamixel import MX64Module
from joy.mxr import ServoWrapperMX

#v07

"""
SERVO_NAMES = {
   0x0E: 'FL', 0x0C: 'ML', 0x2E: 'HL',
   0x08: 'FR', 0x3C: 'MR', 0x04: 'HR'
}
"""

SERVO_NAMES = {
   0x04: 'FL', 0x06: 'ML', 0x0A: 'HL',
   0x08: 'FR', 0x02: 'MR', 0x0C: 'HR'
}

class Buehler(object):
  def __init__(self, offset, sweep, duty):
    """
    Parameters:
      offset (angle in cycles (1.0 is full turn))
      sweep  (angle in cycles (1.0 if full turn)
      duty   (ratio from 0 to 1)
    """
    self.offset = offset
    self.sweep = sweep
    self.duty = duty
    self.resOfs = 0
    self.resOfs = self.at(0)

  def at(self,phi):
    """
    Implements Buehler clock and returns phase as a function of time.
      INPUTS:                                       
        phi -- phase in cycle 0.0 to 1.0
      OUTPUT:                                                                              
        res -- output angle in cycles  (0.0 to 1.0) pi                            
      NOTE: 0 is always pinned to producing an output of 0                         
    """
    phi = (phi+1-self.offset) % 1.0
    if phi<self.duty:
      res = phi * (self.sweep / self.duty)
    else:
      res = (phi-self.duty)*((1-self.sweep) / (1-self.duty))+self.sweep       
    return (res+1-self.resOfs) % 1.0

class TurnInPlace(Plan):
    def __init__(self,app,legs):
        Plan.__init__(self,app)
        self.legs = legs
        self.turn = 0.2

    def behavior(self):
        bias = asfarray([0,0.03,0,0.03,0,0]) # asfarray([0,0.1,0,0,0.1,0])
        liftCoef = asfarray([-1,0,1,-1,0,1])
        for leg,c,b in zip(self.legs,liftCoef, bias):
            leg.set_ang(c*-0.3+b)
        yield self.forDuration(0.3)
        self.legs[1].set_ang(self.turn * 0.5 + bias[1])
        self.legs[4].set_ang(-self.turn * 0.5 + bias[4])
        yield self.forDuration(0.2)
        self.legs[1].set_ang(self.turn+bias[1])
        self.legs[4].set_ang(-self.turn+bias[4])
        yield self.forDuration(0.2)
        for leg,c in zip(self.legs,liftCoef):
            leg.set_ang(0)
        yield self.forDuration(0.3)

    def setTurn(self, t):
        self.turn = t

class Walk(Plan):
    def __init__(self,app):
        Plan.__init__(self,app)
        self.dur = 1
        
    def behavior(self):
        legs = self.app.leg
        walk = self.app.fcp
        # Get legs ready for walking
        progress("(say) preparing to walk")
        goal = asfarray([0,0.5]*3)
        def knot(scl):
          for leg,ga in zip(legs,goal*scl):
            leg.set_ang(ga)
          ang = asfarray([leg.get_ang() for leg in legs])
          progress("... at "+str(ang.round(2)))
          progress("... to "+str((goal*scl).round(2)))
          err = ang-(goal*scl)
          return all(abs(err)<0.1)    
        while not knot(0.5):
          progress("... prep 1 ...")
          yield self.forDuration(0.1)
        while not knot(1.0):
          progress("... prep 1 ...")
          yield self.forDuration(0.1)
        # Start walking
        progress("(say) walking")
        walk.resetPhase()
        yield walk

class Stand(Plan):        
    def behavior(self):
      # Need to break self.app encapsulation, or write ton more code
      if self.app.walk.isRunning():
        progress("(say) halting")
        self.app.halt = True
        while any(self.app.moving):
          progress('...wait...' +str(self.app.moving))
          yield self.forDuration(0.1)
      progress("(say) standing")
      for leg in self.app.leg:
          leg.set_ang(0)
      yield self.forDuration(0.5)

### TODO (Revzen): code up general transition manager

class SCMHexApp(JoyApp):
    def __init__(self, *arg, **kw):
        JoyApp.__init__(self, *arg, **kw)
        self.bueh = Buehler(-.3, 0.8, 0.8 )

    def onStart(self):
        global off
        off = self.robot.off
        #DEBUG.extend(list('Fr'))
        self.T0 = self.now
        self.leg = [
            ServoWrapperMX(self.robot.at.FL, logger=self.logger, ori=-1),
            ServoWrapperMX(self.robot.at.ML, logger=self.logger, ori=-1),
            ServoWrapperMX(self.robot.at.HL, logger=self.logger, ori=-1),
            ServoWrapperMX(self.robot.at.FR, logger=self.logger ),
            ServoWrapperMX(self.robot.at.MR, logger=self.logger ),
            ServoWrapperMX(self.robot.at.HR, logger=self.logger ),
        ]
        self.fcp = FunctionCyclePlan(self, self._fcp_fun, 128, maxFreq=1.75, interval=0.05)
        self.walk = Walk(self)
        self.stand = Stand(self)
        self.freq = 5/60.0
        self.turn = 0
        self.Kturn = 0.24  #0.12 original
        self.rate = 0.05
        self.limit = 1 / 0.30
        self.moving = asfarray([1] * 6)
        self.halt = 1
        self.isCtrlTime = self.onceEvery(0.1)
        self.ctrlQueue = [ l for l in self.leg ]
        self.fcp.setPeriod(1/self.freq)
        self.tip = TurnInPlace(self, self.leg)
        self.cmd = None

    def _fcp_fun(self, phase):
        # Legs array is FL ML HL FR MR HR
        aDes = asfarray([0,0.5,0,0.5,0,0.5]) + phase
        if self.halt:
            # elements close to zero angle stop moving
            self.moving[abs((aDes+.1) % 1.0)<.2]=0
        else:
            self.moving[:]=1
        	
        # radii of the leg midstance from centre of rotation
        radii = asfarray([1.2, -1, 1.2, -1.2, 1, -1.2])
        # Turning influence
        tInf = self.turn * self.Kturn * radii * sin(aDes * 2* pi)
        assert all(abs(tInf)<0.15), "Sanity check on turn influence"
        # progress("inf "+str(tInf))
		    #tInf = self.turn * self.Kturn * asfarray([-1,-1,-1,0,0,0])#1,-1,1,-1,1,-1
        goal = [ self.bueh.at(des) for des in self.moving * (aDes + tInf) % 1.0 ]
        for leg, ga in zip(self.leg, goal):
            leg.set_ang(ga)
        progress( ("TRI goal %.3f" % self.fcp.phase)+"\t".join([ "%+4.2f" % v for v in goal]))#, sameLine = True)

    def onEvent(self,evt):
        try:
            return self._onEvent(evt)
        except KeyboardInterrupt:
            progress('(say) EMERGENCY STOP')
            self.stop()
        except Exception,ex:
            progress(str(ex))
            self.stop()
            
    def _onEvent(self, evt):
        if self.now-self.T0 > 300: # Controller time limit
            self.stop()
        # If time for controller, do round-robin on leg control
        if self.isCtrlTime():
            for i in xrange(3):
                l = self.ctrlQueue.pop(0)
                l.doCtrl()
                self.ctrlQueue.append(l)
        event = None
        if evt.type == KEYDOWN:
           event = evt.key
        if event is not None:
            if event in [K_q, K_ESCAPE] or event == 8:  # 'q' and [esc] stop program
                self.stop()
                #
            elif event == K_SPACE or event == 2:  # [space] stops cycles
                self.fcp.setPeriod(0)
                self.turn = 0
                progress('Period changed to %s' % str(self.fcp.period))
                #
            elif event == K_h:
                self.halt = (self.halt == 0)
                if self.halt:
                  if self.walk.isRunning():
                      self.walk.stop()
                  self.stand.start()  
                else:
                  if self.stand.isRunning():
                      self.stand.stop()
                  self.walk.start()
                 
            elif event == K_s:
                if self.fcp.isRunning():
                  progress("(say) stopping first")
                  self.stand.start()
                else:
                  progress("(say) Enter position commands")
                  # Get a list from the use and complete it to 6 entries by appending zeros
                  self.cmd = list(input("Positions (as 6-tuple, or [] to return to normal mode):"))
                  if self.cmd:
                    for l,a in zip(self.leg, self.cmd):
                      l.set_ang(a)
                  progress("Set to:" + str(self.cmd))
                
            elif event == K_z or event == K_x:
                if any(self.fcp.isRunning()):
                    self.stand.start()
                    progress("Standing first")
                else:
                    if event == K_z:
                        self.tip.setTurn(-0.2)
                    else:
                        self.tip.setTurn(0.2)
                    self.tip.start()
            elif event in (K_UP, K_DOWN) or event in (12, 14):
                f = self.freq                
                if event == K_UP or event == 12:
                    #f = (1 - self.rate) * f + self.rate * self.limit
                    f += 0.02
                else:
                    #f = (1 - self.rate) * f - self.rate * self.limit
                    f -= 0.02
                f = clip(f, -1.5, 1.5)
                if abs(f) < 0.1: # too slow is "stopped"
                    self.fcp.setPeriod(0)
                    progress('(say) stop')
                else:
                    self.fcp.setPeriod(1 / f)
                    if f>0:
                        progress('(say) forward')
                    else:
                        progress('(say) back')                    
                self.freq = f
                progress('Period changed to %g, %.2f Hz' % (self.fcp.period, f))
            elif event in (K_LEFT, K_RIGHT) or event in (13, 15):
				      dTurn = 1 if event in (K_LEFT,13) else -1
				      self.turn = clip(self.turn + dTurn * 0.1, -1, 1)
				      progress('Turn is %.2f' % self.turn)
	    return
        if evt.type not in [TIMEREVENT, JOYAXISMOTION, MOUSEMOTION]:
            JoyApp.onEvent(self, evt)

if __name__ == '__main__':
    print """
  SCM Hexapod controller
  ----------------------
  
  When any key is pressed, starts a FunctionCyclePlan that runs the SCM
  hexapod. Usually works in position mode, but crosses the dead zone by
  switching between position and speed control, giving approximate RPM
  command, and polling for the motor to be out the other side
  
  h -- reset stance
  q -- quit
  arrow keys -- speed/slow and turn
  SPACE -- pause / resume
  any other key -- start 
  
  The application can be terminated with 'q' or [esc]
  """

    import ckbot.dynamixel as DX
    import ckbot.logical as L
    #import ckbot.nobus as NB
    import joy

    if 1:  # 1 to run code; 0 for simulation
        # for actual operation use w/ arch=DX & NO required line:
        L.DEFAULT_BUS = DX
        app = SCMHexApp(
            cfg = dict( logFile = "/tmp/log", logProgress=True ),
            robot=dict(arch=DX, count=6, names=SERVO_NAMES,
#                       port=dict(TYPE='TTY', glob="/dev/ttyACM*", baudrate=115200)
                       port=dict(TYPE='TTY', glob="/dev/ttyUSB*", baudrate=115200)
)
        )
    #else:
    #    L.DEFAULT_BUS = NB
    #    app = SCMHexApp(
    #        robot=dict(
    #            arch=NB, count=len(SERVO_NAMES), fillMissing=True,
    #            required=SERVO_NAMES.keys(), names=SERVO_NAMES
    #        ))

    joy.DEBUG[:] = []
    app.run()


'''
# debug code plotting
import joy
l = asfarray( [ (e['TIME'], e['ang'], e['nid']) for e in joy.loggit.iterlog('/tmp/log.gz') if e['TOPIC'] == "set_ang" ] )
g = asfarray( [ (e['TIME'], e['ang'], e['nid']) for e in joy.loggit.iterlog('/tmp/log.gz') if e['TOPIC'] == "get_ang" ] )
for k in set(unique(l[:,2])):
    plot( g[g[:,2]==k,0], g[g[:,2]==k,1], '+' )
    plot( l[l[:,2]==k,0], l[l[:,2]==k,1], '.' )
'''
