"""
FILE: scratchapp.py

High level classes for making a Scratch-friendly JoyApp

By default, a ScratchApp will map all servos to be accessible from Scratch.

Scratch can write values to <<-servo-name->>.<<-var->>. Servo names follow those in the .at member of the robot Cluster.

See ServoWrapper class for possible <<-var->> values
"""
from joy.decl import *
from joy import JoyApp, Plan, DEBUG
from ckbot.logical import ModulesByName
from ckbot.ckmodule import AbstractServoModule

class ServoWrapper(Plan):
    """
    class ServoWrapper

    Wraps a ServoModule for use from Scratch.

    Theory of operation:
      To reduce the need for realtime communication, Scratch can set a
      "2nd order hold", i.e. a linear segment to be followed by a servo.
      This is done by setting .pos to the segment end position, and setting
      .time to the duration (in mSec) for getting there.
      Angles can be set to arbitrary units with a .amp amplification factor
      and a .ofs offset value.

      By default, angles are given in degrees.

    TODO: add support for go_slack; position reporting
    """
    def __init__(self, app, module, amp=100.0, ofs=0):
        """
        Attributes:
            amp -- float -- amplitude (default 100.0)
            ofs -- float -- offset (default 0)
            p -- float -- planned position (Scratch units)
            p0 -- float -- initial postion (Scratch units)
            t0 -- float -- start time of current motion plan
            v0 -- float -- velocity of currect motion plan
            pos -- float -- desired position at knot point (Scratch units)
            time -- float -- desired duration time of knot point (msec)
        """
        Plan.__init__(self, app)
        self.module=module
        self.amp=amp
        self.ofs=ofs
        self.pos=0
        self.time=None
        self.p0=None
        self.pos=None
        self.now=self.app.now

    def itch(self):
        """
        The opposite of Scratch -- release scratch control
        """
        self.time = None

    def set_spos(self,pos):
        """
        Set position in Scratch units
        """
        return self.module.set_pos(pos*self.amp+self.ofs)

    def get_spos(self):
        """
        Get position in Scratch units
        """
        return (self.module.get_pos()-self.ofs)/self.amp

    def behavior(self):
        while True:
            yield
            # Plan should stop if time is set to None
            if self.time is None:
                return
            now = self.app.now
            self.p=self.p0+self.v0*(now-self.t0)
            self.set_spos(self.p)
            # If we reached the goal point --> stop forever
            if now>(self.t0+self.time):
                self.endMotion()

    def endMotion(self):
        """
        Stop at the end of the current motion segment
        """
        self.set_spos(self.pos)
        self.v0=0
        self.t0=self.app.now
        self.p0=self.pos
        self.time=1e99

    def planMotion(self):
        """
        Construct motion plan from current Scratch variables
        """
        self.p0=self.get_spos()
        self.v0=(self.pos-self.p0)/self.time
        self.t0=self.app.now
        if 'S' in DEBUG:
            progress("%s scr: p0=%6.2g v0=%6.2g" % (self.module.name,self.p0,self.v0))
        if not self.isRunning():
            if 'S' in DEBUG:
                progress("%s starting scratch wrapper" %self.module.name)
            self.start()

    def scrSet(self, var, value):
        """
        Process a command from scratch
        """
        if not var in {'amp', 'ofs', 'pos', 'time'}:
            raise NameError("'%s', Scratch vars should be 'amp', 'ofs', 'pos' or 'time'" % var)
        value = float(value)
        if var != 'time':
            setattr(self, var, value)
        else: # setting 'time'
            if value<=0:
                return
            self.time = float(value)
            self.planMotion()

class ScratchApp( JoyApp ):
    def __init__(self,*arg,**kw):
        if "scr" not in kw:#not kw.has_key("scr"):
            kw.update(scr={})
        JoyApp.__init__(self,*arg,**kw)
        self.sm = ModulesByName()

    def onStart(self):
        """
        Adds Scratch interface wrappers for all .robot servo modules
        NOTE: make sure to call this at the start of subclass .onStart()
        """
        JoyApp.onStart(self)
        for m in self.robot.itermodules():
            if not isinstance(m,AbstractServoModule):
                progress("Warning: no Scratch wrapper available for "+str(m))
                continue
            self.sm._add(m.name, ServoWrapper(self, module=m))

    def onScratchEvent(self, evt):
        """
        Process a SCRATCHUPDATE event.

        Normally, this is called from .onEvent() when a SCRATCHUPDATE event is
        is identified.
        """
        assert evt.type==SCRATCHUPDATE
        nm,var = (evt.var.split(".")+[None,None])[:2]
        if var is None:
            progress("Scratch sent '%s=%s'; can't handle" % (evt.var,evt.value))
            return
        if not hasattr(self.sm,nm):
            progress("Scratch addressed unknown servo '%s'" % nm)
            return
        sw = getattr(self.sm,nm)
        sw.scrSet(var,evt.value)

    def onEvent(self,evt):
        """
        Default event handler -- process all Scratch updates and punt the rest
        to JoyApp superclass handler
        """
        if evt.type == SCRATCHUPDATE:
            return self.onScratchEvent(evt)
        # Punt to superclass
        return JoyApp.onEvent(self,evt)
