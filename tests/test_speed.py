from joy.decl import *
from joy import JoyApp
from time import time as now
from joy.plans import Plan

class TstPlan( Plan ):
  def behavior(self):
    self.tp = []
    while True:
      yield
      self.tp.append(now())

class TstApp( JoyApp ):
  def onStart(self):
    progress("Use 'export PYGIXSCHD=FORCE-ASAP' (virtual time) or 'export PYGIXSCHD=FORCE-FAST' (non-pygame) to use a different scheduler'")
    progress("Hit q to stop; then it will print log of all behavior time iterations; otherwise stops in 10 seconds")
    self.tst = TstPlan(self)
    self.tst.start()
    self.T0 = self.now

  def onStop(self):
    p= self.tst.tp[0]
    for t in self.tst.tp[1:]:
      print "TS %.9f %.8f"%(t,t-p)
      p = t

  def onEvent(self,evt):
    if self.now > self.T0 + 10:
      self.stop()
    return JoyApp.onEvent(self,evt)

A = TstApp()
A.run()

