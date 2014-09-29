from joy.decl import *
from joy import JoyApp, FunctionCyclePlan, progress, DEBUG
from numpy import nan, asfarray, prod, isnan, pi, clip, sign
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
    def __init__(self, servo, **kw):
        assert isinstance(servo, MX64Module), "only works with MX"
        self.servo = servo
        self.aScl = 36000.0  # servo units per rotation
        self.rpmScl = 1 / 64.0
        self.posOfs = 0
        self.ori = 1
        self.desAng = 0
        self.desRPM = 0.1
        self.Kp = 1
        self.Kv = 0.03
        self._clearV()
        self._v = nan
        self.__dict__.update(kw)
        self._ensure_motor = self._set_motor
        self._ensure_servo = self._set_servo

    def _doCtrl(self):
        """execute an interaction of the controller update loop"""
        a = exp(1j * self.get_ang())
        a0 = exp(1j * self.desAng)
        lead = a / a0
        if lead.real < 0.7:
            # outside of capture range; ignore
            return
        pFB = clip(self.Kp * lead.imag, -0.5, 0.5)
        if isnan(self._v):
            vFB = 0
        else:
            vFB = self.Kv * (self._v - self.desRPM)
        rpm = self.desRPM - pFB - vFB
        progress("FB desRPM %g p %g v %g" % (self.desRPM, pFB, vFB))
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
        sleep(2)

    def set_abs_ang(self, ang):
        self._set_servo()
        pos = ang * self.ori * self.aScl
        self._set_pos(pos)
        self._set_motor()


    def set_ang(self, ang):
        self.desAng = ang
        self._doCtrl()
        if "s" in DEBUG:
            progress("%s.set_angle(%g)" % (self.servo.name, ang))

    def get_ang(self):
        pos = self.servo.get_pos()
        ang = (pos - self.posOfs) / self.aScl / self.ori
        if "g" in DEBUG:
            progress("%s.get_angle --> %g" % (self.servo.name, ang))
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
        tq = clip(self.rpmScl * self.ori * rpm, -0.92, 0.92)
        if "r" in DEBUG:
            progress("%s.set_torque(%g) <-- rpm %g" % (self.servo.name, tq, rpm))
        self.servo.set_torque(tq + sign(tq) * 0.08)


class SCMHexApp(JoyApp):
    def __init__(self, *arg, **kw):
        JoyApp.__init__(self, *arg, **kw)

    def onStart(self):
        DEBUG.extend(list('vg'))
        self.leg = [
            ServoWrapperMX(self.robot.at.FL, ori=-1),
            ServoWrapperMX(self.robot.at.ML, ori=-1),
            ServoWrapperMX(self.robot.at.HL, ori=-1),
            ServoWrapperMX(self.robot.at.FR),
            ServoWrapperMX(self.robot.at.MR),
            ServoWrapperMX(self.robot.at.HR),
        ]
        self.triL = self.leg[0::2]
        self.triR = self.leg[1::2]
        self.fcp = FunctionCyclePlan(self, self._fcp_fun, 24, maxFreq=0.5, interval=0.05)
        self.freq = 0.15
        self.rate = 0.05
        self.limit = 1 / 0.45
        # set motors at initial position
        for leg in self.triL:
            leg.set_abs_ang(.25)
        for leg in self.triR:
            leg.set_abs_ang(-.25)

        sleep(5)

        for leg in self.triL + self.triR:
            leg.set_rpm(32)

    def _leg_to_phase(self, leg, phase):
        """
      Move a leg to a given phase, in position mode      
      """
        # if away from dead-zone phase
        if abs(phase - 0.5) > 0.1:
            # If leg might still be in dead zone --> poll it
            if leg.isInDZ:
                leg.get_pos()
            # If outside dead zone --> use position control
            if not leg.isInDZ:
                leg.set_ang((phase - 0.5) * 36000)
        else:  # else --> phase in dead zone range; switch to RPM control
            # If not moving --> abort this
            if self.fcp.period == 0:
                return
            rpm = 60 / self.fcp.period
            leg.set_rpm(rpm)

    def _fcp_fun(self, phase):
        # Loop over all legs
        v = []
        vl = []
        p = []
        # Desired angle for left and right tripods
        aL = phase - 0.5
        aR = ((phase + 0.5) % 1.0) - 0.5
        aDes = [aL, aL, aL, aR, aR, aR]
        for leg, des in zip(self.triL + self.triR, aDes):
            leg.set_ang(des)


    def onEvent(self, evt):
        if evt.type == KEYDOWN:
            if evt.key in [ord('q'), 27]:  # 'q' and [esc] stop program
                self.stop()
                #
            elif evt.key == K_SPACE:  # [space] stops cycles
                self.fcp.setPeriod(0)
                progress('Period changed to %s' % str(self.fcp.period))
                #
            elif evt.key in (K_COMMA, K_PERIOD):
                f = self.freq
                # Change frequency up/down in range -limit..limit hz
                if evt.key == K_COMMA:
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
            elif evt.key == K_h:
                progress(
                    "HELP: ',' to decrease/reverse period; '.' opposite; ' ' stop; 'q' end program; 'h' this help; other keys start Plan")
                #
            else:  # any other key
                progress("Starting cycles....")
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
  , and . -- change rate
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

