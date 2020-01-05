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
      and a .ofs offset value. The update rate for motors can be adjusted using
      the .rate member.

      Once the motion is complete, it is replaced with repeating a set_pos for
      the final position indefinitely.

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
            time -- int -- desired duration time of knot point (msec)
            rate -- int -- update rate for motor commands (msec)
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
        self.rate=100

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
            yield self.forDuration(self.rate * 1e-3)
            # Plan should stop if time is set to None
            if self.time is None:
                return
            now = self.app.now
            self.p=self.p0+self.v0*(now-self.t0)
            self.set_spos(self.p)
            # If we reached the goal point --> stop forever
            if (now-self.t0)*1000>self.time:
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
        try:
            self.p0=self.get_spos()
            self.v0=1000*(self.pos-self.p0)/float(self.time)
        except ZeroDivisionError:
            return
        except TypeError:
            return
        self.t0=self.app.now
        if 'S' in DEBUG:
            progress("%s plan: p0=%6.2g p1=%6.2g v0=%6.2g T=%6.2g" % (self.module.name,self.p0,self.pos,self.v0,self.time * 1e-3))
        if not self.isRunning():
            if 'S' in DEBUG:
                progress("%s starting scratch wrapper" %self.module.name)
            self.start()

    def scrSet(self, var, value):
        """
        Process a command from scratch
        """
        if not var in {'amp', 'ofs', 'pos', 'time','mode'}:
            raise NameError("'%s', Scratch vars should be 'amp', 'ofs', 'pos','mode' or 'time'" % var)
        if 'S' in DEBUG:
            progress("%s set: %s = %6g" % (self.module.name,var,float(value)))
        if var == 'mode':
            if int(value) in [0,1,2]:
                self.module.set_mode(int(value))
            return
        setattr(self, var, float(value))

class ScratchApp( JoyApp ):
    """
    concrete class ScratchApp extends JoyApp

    A ScratchApp automatically wraps all servos with ServoWrapper instances,
    and processes scratch events to give them commands. The scratch
    events take on the form of <<module>>.<<property>> sensors set to values.
    The .time property must be set last, to trigger creation of the second order
    hold for that motor.
    In addition, all commands from scratch are queued until the special sensor "GO"
    is given a value; at that time all queued commands are processed.
    """
    def __init__(self,*arg,**kw):
        if not kw.has_key("scr"):
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
        self.q = []

    def onScratchEvent(self, evt):
        assert evt.type==SCRATCHUPDATE
        if evt.var != "GO":
            self._enQueueEvt(evt)
            return
        ### --> 'GO' event received
        act = set()
        # Process all the settings
        for sw,var,val in self.q:
            sw.scrSet(var,val)
            # If '.time' was set --> we will need to activate this wrapper
            if var == 'time':
                act.add(sw)
        # Activate all wrappers that needed activation
        for sw in act:
            sw.planMotion()
        self.q = []

    def _enQueueEvt(self, evt):
        """ *PRIVATE*
        Parse a SCRATCHUPDATE event and put on queue if valid.

        Normally, this is called to error check Scratch events before they are
        put in the queue.
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
        self.q.append( (sw,var,evt.value) )

    def onEvent(self,evt):
        """
        Default event handler -- process all Scratch updates and punt the rest
        to JoyApp superclass handler
        """
        if evt.type == SCRATCHUPDATE:
            return self.onScratchEvent(evt)
        # Punt to superclass
        return JoyApp.onEvent(self,evt)
