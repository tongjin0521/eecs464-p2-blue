# -*- coding: utf-8 -*-
"""
Created on Thu Nov  3 17:05:26 2016

@author: birds
"""
DEBUG = []
from time import time as now
from ckbot.dynamixel import MX64Module
from numpy import nan,exp,angle,abs,clip,pi,isnan,sign
from plans import Plan

class CRMXServoPlan(Plan):
    """
    Concerete class CRMXServoPlan
    
    This joy.Plan subclass implements a lightweight and efficient
    control scheme allowing MX64 modules in Continuous Rotation (CR)
    configuration to be used with position control. 
    
    Typical usage:

    >>> class App(JoyApp):
    >>>    def onStart(self):
    >>>        self.smx = CRMXServoPlan(self,self.robot.itervalues().next())
    >>>        self.smx.start()
    >>>        self.when = self.onceEvery(1)
    >>>    def onEvent(self,evt):
    >>>        if self.when():
    >>>          self.smx.set_rot((self.now % 6)/6.0)
    >>> App().run()

    This example takes the first module found, assumes it is an MX64 and
    makes it rotate indefinitely in steps of 60 degrees each
    """
    def __init__(self, app, servo, **kw):
        """
        INPUT:
          app -- JoyApp -- App that is running this Plan
          servo -- MX64Module -- module to be wrapped with controller
          **kw -- additional attribute variable overrides as 
               specified below.
               
        ATTRIBUTES:
           rpmScl -- float -- conversion factor RPM to MX64 set_torque()
           posOfs -- float [rotations] -- zero position
           ori -- int -- orientation (+1 or -1)
           rate -- float [sec] -- update rate for controller
           waitRate -- float [sec] -- polling rate for position reading
           desAng -- float [rotations] -- initial desired angle
           Kp -- float -- proportional gain
           minFB -- float -- minimal magnitude cutoff for servo.set_torque()
                             (don't make this less than 0.05!)
        """
        Plan.__init__(self,app,**kw)
        # Units for RPM
        self.rpmScl = 1 / 64.0
        # Position offset of zero position
        self.posOfs = 0
        # Orientation of motor; change to -1 to flip
        self.ori = 1
        # Update rate [sec] -- time to sleep between controller updates
        self.rate = 0.1
        # Retry rate [sec] waiting for comms updates 
        self.waitRate = 0.02
        # Current desired angle
        self.desAng = 0
        # Proportional gain
        self.Kp = 25.0
        # Minimal feedback cutoff (don't make this less than 0.05!)
        self.minFb = 0.1
        # Override defaults from keyword arguments
        self.__dict__.update(kw)
        ### 
        ### Internal variables
        ###
        assert isinstance(servo, MX64Module), "only works with MX"
        self.servo = servo
        # Angle units per servo rotation
        self.aScl = 36000.0  
        self._rawAng = None
        self._p = [ RuntimeError("trigger initial request") ]
        self._ensure_motor = self._set_motor
        self._ensure_servo = self._set_servo

    def behavior(self):
        """controller update thread"""
        while True:
          # If response hasn't arrived yet -- wait
          if not self._p:
            yield self.forDuration(self.waitRate)
            continue
          # If response received -- update angle estimate 
          if not isinstance(self._p[0],Exception):
            self._rawAng = self._p[0]
          else: # was a communication error -- warn
            progress("%s comm error %s" % (self.servo.name,str(self._p[0])))
          # send out a new position request
          self._p = self.servo.get_pos_async()
          ##
          ## Control loop iteration
          ##
          # Current angle phasor
          a = exp(1j * self.get_ang()*2*pi)
          # Desired angle phasor
          a0 = exp(1j * self.desAng*2*pi)
          lead = angle(a / a0)
          # Compute proportional feedback
          fb = clip(-self.Kp * lead, -45, 45)
          # Only give feedback commands if they are sufficiently large to matter
          rpm = fb if abs(fb)<self.minFb else 0
              rpm = 0
          if "F" in DEBUG:
              progress("FB %g lead %g at %g" % (rpm,lead,angle(a)))
          # Push into the motor
          self._set_rpm(rpm)
          yield self.forDuration(self.rate)
          
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

    def set_ang(self, ang):
        """
        Set the position, in units of rotation (1.0 for full rotation)
        """
        self.desAng = ang
        if "s" in DEBUG:
            progress("%s.set_ang(%g)" % (self.servo.name, ang))

    def get_ang(self):
        """
        Get the (most recently read) position, in units of rotation (1.0 for full rotation)
        """
        ang = self.servo.dynamixel2ang(self._rawAng) / self.aScl / self.ori - self.posOfs
        if "g" in DEBUG:
            progress("%s.get_ang() --> %g" % (self.servo.name, ang))
        return ang

    def go_slack(self):
        """
        Make the servo go slack
        """
        return self.servo.go_slack()
                
    def _set_rpm(self, rpm):
        """(private) Push an RPM setting to the motor"""
        self._ensure_motor()
        tq = clip(self.rpmScl * self.ori * rpm, -0.999, 0.999)
        if "r" in DEBUG:
            progress("%s.set_torque(%g) <-- rpm %g" % (self.servo.name, tq, rpm))
        self.servo.set_torque(tq)

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
        # Use this rpm in dead zone
        self.deadRPM = 10
        self.deadZone = 0.10
        self.punch = 2.0
        self.Kp = 10
        self.logger = logger
        self._clearV()
        self._v = nan
        self.__dict__.update(kw)
        self._ensure_motor = self._set_motor
        self._ensure_servo = self._set_servo

    def _ang2srv(self,ang):
        """
        Convert cycle angle (range 0..1) to servo position
        
        Angle 0 is mapped to .posOfs, growing in direction .ori
        """
        return int((((float(ang)-self.posOfs)*self.ori) % 1.0) * self.aScl)
    
    def _srv2ang(self,srv):
        """
        Convert cycle angle (range 0..1) to servo position
        
        Angle 0 is mapped to .posOfs, growing in direction .ori
        """
        return float(srv)/self.aScl/self.ori + self.posOfs
        
    def doCtrl(self):
        """execute an interaction of the controller update loop"""
        a = exp(1j * self.get_ang()*2*pi)
        a0 = exp(1j * self.desAng*2*pi)
        lead = angle(a / a0)
        if abs(lead)>0.7*pi:
            pFB = 0
            pCH = 0
            rpm = self.deadRPM
            if "F" in DEBUG:
                progress("FB lead %5.2g out of range" % (lead))
                    # outside of capture range; ignore
        else:
            pCH = (self.punch * sign(lead)) if abs(lead)>self.deadZone else 0
            pFB = clip(pCH + self.Kp * lead, -45, 45)
            rpm = self.desRPM - pFB
            if abs(rpm)<0.1:
                rpm = 0
            if "F" in DEBUG:
                progress("FB desRPM %g p %g c %g" % (self.desRPM, pFB, pCH))
        if self.logger:
            self.logger.write( "ctrl", nid=self.servo.node_id,  
                              desRPM=str(self.desRPM), pFB=str(pFB), pCH=str(pCH) )
        # Push into the motor
        self._set_rpm(rpm)

    def _set_servo(self):
        """(private) set module in servo mode

            also configures the _ensure_* callbacks so that _ensure_* only
            sends command to motor once
        """
        self.servo.set_mode(0)
        if "h" in DEBUG:
            progress("%s.set_mode('Servo')" % (self.servo.name))
        if self.servo.get_mode() == 0:
            self._ensure_servo = lambda: None
        self._ensure_motor = self._set_motor

    def _set_motor(self):
        """(private) set module in motor mode

            also configures the _ensure_* callbacks so that _ensure_* only
            sends command to motor once
        """
        self.servo.set_mode(1)
        if "h" in DEBUG:
            progress("%s.set_mode('Motor')" % (self.servo.name))
        if self.servo.get_mode() == 1:
            self._ensure_motor = lambda: None
        self._ensure_servo = self._set_servo

    def set_ang(self, ang):
        self.desAng = float(ang) %1.0
        if "s" in DEBUG:
            progress("%s.set_angle(%g)" % (self.servo.name, ang))
        if self.logger:
            self.logger.write( "set_ang", nid=self.servo.node_id, ang=str(ang) )

    def get_ang(self):
        ang = self._srv2ang(self.servo.get_pos())
        if "g" in DEBUG:
            progress("%s.get_angle --> %g" % (self.servo.name, ang))
        if self.logger:
            self.logger.write( "get_ang", nid=self.servo.node_id, ang=str(ang) )
        self._updateV(ang)
        return ang

    def get_v(self):
        """
        Get a velocity estimate
        """
        return self._v
        
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

    def _set_rpm(self, rpm):
        """Push an RPM setting to the motor"""
        self._ensure_motor()
        tq = clip(self.rpmScl * self.ori * rpm, -0.999, 0.999)
        if "r" in DEBUG:
            progress("%s.set_torque(%g) <-- rpm %g" % (self.servo.name, tq, rpm))
        self.servo.set_torque(tq)

if __name__=="__main__":
    import ckbot.logical as L
    from time import sleep
    T0 = now()
    def progress(msg,sameLine=False):
        print "%5.3g " % (now()-T0),msg,
        if not sameLine:
            print
    c = L.Cluster(count=1)
    m = c.values()[0]
    w = ServoWrapperMX(m,ori=-1,posOfs=0.5)
    try:
      from pylab import linspace
      a = linspace(-2,2,103)
      b = [ w._srv2ang(w._ang2srv(ai)) for ai in a ]
      assert abs((a-b+.2)%1.0-.2).max() < 0.01
    except ImportError:
      print "WARNING: No pylab for error estimate test"
    DEBUG[:] = ['h','F','r','s']    
    last = now()
    while 1:
        w.doCtrl()
        if now()-last>1:
            goal = 0.3 * (long(now()) % 2)
            w.set_ang( goal )
            last = now()
            progress("goal %g" % goal)
        sleep(0.3)
        