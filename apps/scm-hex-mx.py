from joy.decl import *
from joy import JoyApp, FunctionCyclePlan, progress, DEBUG
from numpy import nan, asfarray, prod, isnan, pi, clip, sign, angle
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
    0x0F: 'FL', 0x12: 'ML', 0x16: 'HL',
    0x17: 'FR', 0x0A: 'MR', 0x11: 'HR'
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
        self.Kp = 30
        self.Kv = 0
        self.Kt = 0.1
        self.logger = logger
        self._clearV()
        self._v = nan
        self.__dict__.update(kw)
        self._ensure_motor = self._set_motor
        self._ensure_servo = self._set_servo

    def _doCtrl(self):
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
        self._doCtrl()
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
        self.servo.set_torque(tq + sign(tq) * 0.08)


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
        self.fcp = FunctionCyclePlan(self, self._fcp_fun, 32, maxFreq=0.75, interval=0.05)
        #self.fcp = FunctionCyclePlan(self, lambda ignore : None, 256, maxFreq=0.5, interval=0.01)
        self.freq = 45/60.0
        self.turn = 0
        self.rate = 0.05
        self.limit = 1 / 0.45
        # set motors at initial position
        for leg in self.leg:
            leg.set_ang(.25)

    def _fcp_fun(self, phase): 
        # Desired angle for left and right tripods
        aL = phase - 0.5
        aR = ((phase + 0.5) % 1.0) - 0.5
        aDes = [aL, aL, aL, aR, aR, aR]
        # radii of the leg midstance from centre of rotation
        radii = asfarray([1.2, 1, 1.2, -1.2, -1, -1.2])
        # Turning influence
        tInf = self.Kturn * radii * sin(aDes * pi)
        for leg, des in zip(self.triL + self.triR, aDes+tInf):
            leg.set_ang(des)

    def onEvent(self, evt):
        if self.now-self.T0 > 10:
            self.stop()
        if evt.type == KEYDOWN:
            if evt.key in [ord('q'), 27]:  # 'q' and [esc] stop program
                self.stop()
                #
            elif evt.key == K_SPACE:  # [space] stops cycles
                self.fcp.setPeriod(0)
                progress('Period changed to %s' % str(self.fcp.period))
                #
            elif evt.key in (K_UP, K_DOWN):
                f = self.freq
                # Change frequency up/down in range -limit..limit hz
                if evt.key == K_UP:
                    f = (1 - self.rate) * f - self.rate * self.limit
                else:
                    f = (1 - self.rate) * f + self.rate * self.limit
                if abs(f) < 1.0 / self.limit:
                    self.fcp.setPeriod(0)
                else:
                    self.fcp.setPeriod(1 / f)
                self.freq = f
                progress('Period changed to %g, %.2f Hz' % (self.fcp.period, f))
                #
            elif evt.key in (K_LEFT, K_RIGHT):
                tn = self.turn
                # Change frequency up/down in range -limit..limit hz
                if evt.key == K_LEFT:
                    tn = (1 - self.rate) * tn - self.rate
                else:
                    tn = (1 - self.rate) * tn + self.rate 
                self.turn = tn                    
                progress('Turn changed to %.2f' % (self.turn))
                #
            elif evt.key == K_h:
                progress(
                    "HELP: UP/DOWN arrow for speed; ' ' stop; 'q' end program; 'h' this help; other keys start Plan")
                #
            else:  # any other key
                progress("Starting cycles....")
                self.fcp.setPeriod(1 / self.freq)
                self.fcp.start()
            return  # (from all keypress handlers)
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
    import ckbot.nobus as NB
    import joy

    if 1:  # 1 to run code; 0 for simulation
        # for actual operation use w/ arch=DX & NO required line:
        L.DEFAULT_BUS = DX
        app = SCMHexApp(
            cfg = dict( logFile = "/tmp/log" ),
            robot=dict(arch=DX, count=len(SERVO_NAMES), names=SERVO_NAMES)
        )
    else:
        L.DEFAULT_BUS = NB
        app = SCMHexApp(
            robot=dict(
                arch=NB, count=len(SERVO_NAMES), fillMissing=True,
                required=SERVO_NAMES.keys(), names=SERVO_NAMES
            ))

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
