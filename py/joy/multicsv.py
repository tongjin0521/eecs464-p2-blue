from os.path import getmtime
from misc import loadCSV
from decl import KEYDOWN, progress
from plans import Plan,SheetPlan
from joy import JoyApp, DEBUG

class MultiSheetPlan( Plan ):
    """
    A plan that runs ONE of its subservient SheetPlans
    """
    def __init__(self,app,**kw):
        Plan.__init__(self,app)
        self.subs = {}
        self.run = None

    def addCSV(self,nm,fn):
        if self.subs.has_key(nm):
            raise KeyError("Sheet '%s' already exists" % nm)
        self.subs[nm] = EasyCSV(self.app,fn)

    def start(self,nm):
        if self.isRunning():
            progress("Already running '%s' -- ignored" % self.run)
        self.run = self.subs.get(nm,None)
        if self.run is not None:
            Plan.start(self)
            return True
        return False

    def behavior(self):
        # If there's a plan to be running
        if self.run is not None:
            # Let it check for .csv updates
            self.run.update()
            # --> have it take over my execution slot
            yield self.run.getPlan()

class EasyCSV( object ):
    """
    A callable which handles a .CSV with a SheetPlan
    """
    def __init__(self,app,fn):
        assert isinstance(app,JoyApp)
        self.app = app
        assert fn.upper().endswith(".CSV")
        self.fn = fn
        self.sp = None
        self.mtime = None

    def update(self):
        try:
            mt = getmtime(self.fn)
        except OSError:
            progress("CSV '%s' has no mtime" % self.fn)
            return # no file
        if self.mtime == mt:
            return # no change
        if self.sp and self.sp.isRunning():
            return # defer until no longer running
        sheet = loadCSV(self.fn)
        if self.sp is None:
            self.sp = SheetPlan(self.app,sheet)
        else:
            self.sp.update(sheet)
        self.mtime = mt
        progress("Updated CSV '%s' at %s" % (self.fn,mt))

    def getPlan(self):
        return self.sp

class MultiCSVApp( JoyApp ):
    def onStart(self):
        self.msp = MultiSheetPlan(self)
        progress("Configuring CSV handlers:")
        for k,v in self.__class__.__dict__.iteritems():
            if k.startswith("onKey_") and type(v) is str:
                self.msp.addCSV(k,v)
                progress("\t\t%s --> '%s'" % (k,v))

    def onEvent(self,evt):
        if evt.type == KEYDOWN:
            if self.msp.start("onKey_"+evt.unicode):
                return
        return JoyApp.onEvent(self,evt)
