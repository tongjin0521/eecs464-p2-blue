from sys import stderr
from joy.decl import *
from joy import Plan, JoyApp
from joy.pygix import now, SCHD
from time import time
from joy.pygix.constants import IMPL

progress("The pygix IMPL=%s, SCHD=%s\n" % (repr(IMPL),repr(SCHD)))
tlog = []
class FastPlan(Plan):
  def behavior(self):
    N=0
    while True:
      stderr.write("%6d,%16.6f,%16.6f,%16.6f\n" % (N,time(),self.app.now,now())) 
      N+=1
      tlog.append(self.app.now)
      yield

class App(JoyApp):
  def onStart(self):
    self.fp = FastPlan(self)
    self.fp.start()
  def onStop(self):
    progress("="*40)
    progress("Results in 'tlog' global variable (length=%d)" % len(tlog))
    progress("="*40)

app = App()
app.run()
