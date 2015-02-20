from joy.decl import *
from joy import JoyApp, FunctionCyclePlan, progress, DEBUG, Plan
from numpy import nan, asfarray, prod, isnan, pi, clip, sin, angle, sign, round
from cmath import exp
from time import sleep, time as now
from ckbot.dynamixel import MX64Module

'''
SERVO_NAMES = {
   0x13: 'FL', 0x1E : 'ML', 0x15 : 'HL',
   0x0C: 'FR', 0x14 : 'MR', 0x07 : 'HR'
}
'''
SERVO_NAMES = {
    0x17: 'FL', 0x11: 'ML', 0x0A: 'HL',
    0x12: 'FR', 0x16: 'MR', 0x0F: 'HR'
}


class ServoWrapperMX(object):
    def __init__(self, servo, logger=None, **kw):
        assert isinstance(servo, MX64Module), "only works with MX"
        self.servo = servo
        self.aScl = 36000.0  # servo units per rotation
        self.rpmScl = 1 / 64.0
        self.posOfs = 0
        self.ori = 1
        self.desAng = 0
        self.desRPM = 00
        self.Kp = 25.0
        self.Kv = 0
        self.logger = logger
        self._clearV()
        self._v = nan
        self.__dict__.update(kw)
        self._ensure_motor = self._set_motor
        self._ensure_servo = self._set_servo

    def doCtrl(self):
        """execute an interaction of the controller update loop"""
        a = exp(1j * self.get_ang()*2*pi)
        a0 = exp(1j * self.desAng*2*pi)
        lead = angle(a / a0)
        if abs(lead)>0.7*pi:
            if "F" in DEBUG:
                progress("FB desRPM %g out of range" % (self.desRPM))
                    # outside of capture range; ignore
            return
        pFB = clip(self.Kp * lead, -45, 45)
        if isnan(self._v):
            vFB = 0
        else:
            vFB = self.Kv * (self._v - self.desRPM)
        rpm = self.desRPM - pFB - vFB
        if abs(rpm)<0.1:
            rpm = 0
        if "F" in DEBUG:
            progress("FB desRPM %g p %g v %g" % (self.desRPM, pFB, vFB))
        if self.logger:
            self.logger.write( "ctrl", nid=self.servo.node_id,  
                              desRPM=str(self.desRPM), pFB=str(pFB), vFB=str(vFB) )
        # Push into the motor
        self._set_rpm(rpm)

    def _set_servo(self):
        """(private) set module in servo mode

            also configures the _ensure_* callbacks        
        """
        self.servo.set_mode(0)
        if "h" in DEBUG:
            progress("%s.set_mode('Servo')" % (self.servo.name))
        if self.servo.get_mode() == 0:
            self._ensure_servo = lambda: None
        self._ensure_motor = self._set_motor

    def _set_motor(self):
        """(private) set module in motor mode

            also configures the _ensure_* callbacks        
        """
        self.servo.set_mode(1)
        if "h" in DEBUG:
            progress("%s.set_mode('Motor')" % (self.servo.name))
        if self.servo.get_mode() == 1:
            self._ensure_motor = lambda: None
        self._ensure_servo = self._set_servo

    def _set_pos(self, pos):
        self.servo.set_pos(pos)

    def set_ang(self, ang):
        self.desAng = ang
        if "s" in DEBUG:
            progress("%s.set_angle(%g)" % (self.servo.name, ang))
        if self.logger:
            self.logger.write( "set_ang", nid=self.servo.node_id, ang=str(ang) )

    def get_ang(self):
        pos = self.servo.get_pos()
        ang = (pos - self.posOfs) / self.aScl / self.ori
        if "g" in DEBUG:
            progress("%s.get_angle --> %g" % (self.servo.name, ang))
        if self.logger:
            self.logger.write( "get_ang", nid=self.servo.node_id, ang=str(ang) )
        self._updateV(ang)
        return ang

    def _clearV(self):
        """Clear state of velocity estimator"""
        self._lastA = None
        self._lastT = None

    def _updateV(self, ang):
        """
        Push angle measurement into velocity estimator
        """
        t = now()
        if self._lastA is not None:
            da = ang - self._lastA
            dt = t - self._lastT
            vNew = da / dt * 60.0  # angle is in rotations, dt seconds
            # If no previous estimate --> use current estimate
            if isnan(self._v):
                self._v = vNew
            else:  # Use first order lowpass to smooth velocity update
                a = 0.2
                self._v = self._v * (1 - a) + vNew * a
        self._lastA = ang
        self._lastT = t

    def adjustV(self, ratio):
        """adjust scaling of the velocity to make motors match up"""
        self.rpmScl = clip(self.rpmScl * ratio, 0.3 / 64, 3.0 / 64)

    def set_rpm(self, rpm):
        self.desRPM = rpm
        return self._set_rpm(rpm)

    def _set_rpm(self, rpm):
        """Push an RPM setting to the motor"""
        self._ensure_motor()
        tq = clip(self.rpmScl * self.ori * rpm, -0.999, 0.999)
        if "r" in DEBUG:
            progress("%s.set_torque(%g) <-- rpm %g" % (self.servo.name, tq, rpm))
        self.servo.set_torque(tq ) #+ sign(tq) * 0.08)

class TurnInPlace(Plan):
    def __init__(self,app,legs):
        Plan.__init__(self,app)
        self.legs = legs
        self.turn = 0.2

    def behavior2(self):
        # radii of the leg midstance from centre of rotation
        tDir = asfarray([1,1,1,-1,-1,-1])
        ang = asfarray([0,0,0,1,1,1])
        dur = 1.0
        steps = 10
        for k in xrange(steps):
            ang += tDir / steps
            for leg,a in zip(self.legs,ang):
                leg.set_ang(a)
            yield self.forDuration(dur/steps)

    def behavior(self):
        bias = asfarray([0,0.1,0,0,0.1,0])
        liftCoef = asfarray([-1,0,1,-1,0,1])
        for leg,c,b in zip(self.legs,liftCoef, bias):
            leg.set_ang(c*-0.3+b)
        yield self.forDuration(0.5)
        self.legs[1].set_ang(self.turn * 0.5 + bias[1])
        self.legs[4].set_ang(-self.turn * 0.5 + bias[4])
        yield self.forDuration(0.2)
        self.legs[1].set_ang(self.turn+bias[1])
        self.legs[4].set_ang(-self.turn+bias[4])
        yield self.forDuration(0.2)
        for leg,c in zip(self.legs,liftCoef):
            leg.set_ang(0)
        yield self.forDuration(0.5)

    def setTurn(self, t):
        self.turn = t


class SCMHexApp(JoyApp):
    def __init__(self, *arg, **kw):
        JoyApp.__init__(self, *arg, **kw)

    def onStart(self):
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
        self.triL = self.leg[0::2]
        self.triR = self.leg[1::2]
        self.fcp = FunctionCyclePlan(self, self._fcp_fun, 128, maxFreq=1.75, interval=0.05)
        #self.fcp = FunctionCyclePlan(self, lambda ignore : None, 256, maxFreq=0.5, interval=0.01)
        self.freq = 5/60.0
        self.turn = 0
        self.Kturn = 0.12
        self.rate = 0.05
        self.limit = 1 / 0.30
        self.moving = asfarray([1] * 6)
        self.halt = 1
        # set motors at initial position
        for leg in self.triL:
            leg.set_ang(0)
        for leg in self.triR:
            leg.set_ang(.5)
        self.isCtrlTime = self.onceEvery(0.1)
        self.ctrlQueue = [ l for l in self.leg ]
        self.fcp.setPeriod(1/self.freq)
        self.fcp.start()
        self.tip = TurnInPlace(self, self.leg)

    def _fcp_fun(self, phase): 
        # Desired angle for left and right tripods
        aL = phase - 0.5
        aR = ((phase + 0.5) % 1.0) - 0.5
        aDes = asfarray([aL, aL, aL, aR, aR, aR])
        if self.halt:
            # elements close to zero angle stop moving
            self.moving[(aDes<.1)|(aDes>.9)]=0
        else:
            self.moving[:]=1
		'''	
        # radii of the leg midstance from centre of rotation
        radii = asfarray([1.2, 1, 1.2, -1.2, -1, -1.2])
        # Turning influence
        tInf = self.turn * self.Kturn * radii * sin(aDes * 2* pi)
        assert all(abs(tInf)<0.15), "Sanity check on turn influence"
        # progress("inf "+str(tInf))
		'''
        tInf = self.turn * self.Kturn * asfarray([1,1,1,-1,-1,-1])
        goal = self.moving * (aDes + tInf) % 1.0
        for leg, des in zip(self.triL + self.triR, goal):
            leg.set_ang(des)
        progress( "goal "+"\t".join([ "%+4.2f" % v for v in goal]), sameLine = True)

    def onEvent(self,evt):
        try:
            return self._onEvent(evt)
        except Exception,ex:
            progress(str(ex))
            
    def _onEvent(self, evt):
        if self.now-self.T0 > 120: # Controller time limit
            self.stop()
        # If time for controller, do round-robin on leg control
        if self.isCtrlTime():
            for i in xrange(3):
                l = self.ctrlQueue.pop(0)
                l.doCtrl()
                self.ctrlQueue.append(l)
            # also piggyback some housekeeping
            if not self.fcp.isRunning():
                if not self.tip.isRunning():
                    self.fcp.start()
        if evt.type == KEYDOWN or evt.type == JOYBUTTONDOWN:
            if evt.type == KEYDOWN:
                event = evt.key
            else:
                event = evt.button
            if event in [K_q, K_ESC] or event == 8:  # 'q' and [esc] stop program
                self.stop()
                #
            elif event == K_SPACE or event == 2:  # [space] stops cycles
                self.fcp.setPeriod(0)
                self.turn = 0
                progress('Period changed to %s' % str(self.fcp.period))
                #
            elif event == K_h:
                self.halt = (self.halt == 0)
            elif event == K_z or event == K_x:
                if any(self.moving):
                    halt = 1
                else:
                    self.fcp.stop()
                    if event == K_z:
                        self.tip.setTurn(-0.2)
                    else:
                        self.tip.setTurn(0.2)
                    self.tip.start()
            elif event in (K_UP, K_DOWN) or event in (12, 14):
                f = self.freq                
                if event == K_UP or event == 12:
                    #f = (1 - self.rate) * f + self.rate * self.limit
                    f += 0.2
                else:
                    #f = (1 - self.rate) * f - self.rate * self.limit
                    f -= 0.2
                f = clip(f, -1.5, 1.5)
                if abs(f) < 1.0 / self.limit:
                    self.fcp.setPeriod(0)
                    progress('(say) stop')
                else:
                    self.fcp.setPeriod(1 / f)
                    if f>0:
                        progress('(say) advance')
                    else:
                        progress('(say) retreat')                    
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
  
  h -- help
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
            cfg = dict( logFile = "/tmp/log" ),
            robot=dict(arch=DX, count=len(SERVO_NAMES), names=SERVO_NAMES,
                       port="/dev/ttyACM*")
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
