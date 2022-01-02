from joy.plans import Plan, SheetPlan
from joy import JoyApp
from joy.events import describeEvt
from joy.decl import *

class TestAppMixin( object ):
  def log( self, msg ):
    progress("TEST %s[%04x] : %s"
      % (self.__class__.__name__,id(self),msg))

  def onStart( self ):
    self.log("in onStart()")

  def onStop( self ):
    self.log("in onStop()")

def parrotSays( arg ):
  progress('TEST: quoth Parrot: %s' % repr(arg))

def cansOfSpam( arg ):
  progress('TEST: we have %d cans of spam' % arg)

class BasicPlanTestClass(Plan):
  def __init__(self,app,delay,*arg,**kw):
    Plan.__init__(self,app,*arg,**kw)
    self.delay = delay

  def behavior(self):
    self.app.log("test forDuration(%g)" % self.delay)
    yield self.forDuration(self.delay)
    for k in range(5):
      self.app.log("%x says tic" % id(self))
      yield self.forDuration(0.5)
      self.app.log("%x says toc" % id(self))
      yield self.forDuration(0.5)

class SequencedPlans(Plan):
  def __init__(self,app,*arg,**kw):
    Plan.__init__(self,app,*arg,**kw)
    self.p1 = BasicPlanTestClass(app,1)
    self.p2 = BasicPlanTestClass(app,1)
    self.go = False

  def onEvent( self, evt ):
    if evt.type == TIMEREVENT:
      self.app.log("got TIMEREVENT, p1=%d, p2=%d"
        % (len(self.p1._Plan__evts),len(self.p2._Plan__evts)) )
    elif evt.type == KEYDOWN and evt.key == 97:
      self.app.log(" GO GO GO!")
      self.go = True
    return True

  def behavior(self):
    yield self.forDuration(1)
    while not self.go:
      yield
    self.app.log("first sequence")
    yield self.p1
    self.app.log("second sequence")
    yield self.p2
    self.app.log("end of sequence")

class BasicPlanTestApp( JoyApp, TestAppMixin ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
    TestAppMixin.__init__(self)

  def onStart(self):
    TestAppMixin.onStart(self)
    self.tp = [ BasicPlanTestClass(self,k) for k in range(4) ]
    self.sht = SheetPlan( self, inlineCSV('''
# ------------------------- sheet for parrots and spam --------------------
"t",    "say",      "eat"
1,      "hello",
2,      "hungry",   3
3,      "more!",    2
4,      "tired",
5,      ,           99
# ------------------------- sheet for parrots and spam --------------------
    '''), say=parrotSays, eat=cansOfSpam)
    self.cyc = CyclePlan( self, {
        0.1 : lambda x : parrotSays("at 0.1"),
        0.4 : lambda x : parrotSays("at 0.4"),
        0.6 : lambda x : parrotSays("at 0.6"),
        0.9 : lambda x : parrotSays("at 0.9"),
    } )
    self.seq = SequencedPlans(self)
    self.tp.extend( [self.sht,self.cyc,self.seq ] )

  def onEvent(self,evt):
    cmd = None
    if evt.type == JOYBUTTONDOWN and evt.joy==0:
      if evt.button<len(self.tp):
        cmd = evt.button
    if evt.type == KEYDOWN and evt.key>=49 and evt.key<=49+len(self.tp):
      cmd = evt.key - 49
    if cmd is not None:
      self.log("Starting plan # %d" % cmd)
      self.tp[cmd].start()
      return
    #
    if evt.type==KEYDOWN:
      self.seq.push(evt)
    #
    if evt.type==JOYAXISMOTION and evt.joy==0:
      if evt.axis==0:
        self.sht.setRate(evt.value+1.2)
        self.log("Sheet rate set to %g" % self.sht.rate)
        return
      elif evt.axis==1:
        self.cyc.setPeriod(evt.value*4)
        self.log("Cycle period set to %s" % str(self.cyc.period))
        return
    #
    if evt.type != TIMEREVENT:
      JoyApp.onEvent(self,evt)

class Example(JoyApp):
  def __init__(self,arch):
    JoyApp.__init__(self,robot=dict(arch=arch,required=set([0x3])))

  def onStart(self):
    self.getpos = self.robot.at.Nx03.get_pos
    self.setpos = self.robot.at.Nx03.set_pos

  def onEvent(self,evt):
    progress( describeEvt(evt) )
    if evt.type==JOYAXISMOTION and evt.axis==3:
      pos = evt.value * 5000
      pos = max(min(pos,9000),-9000)
      self.setpos(pos)
    elif evt.type==QUIT or (evt.type==KEYDOWN and evt.key==K_ESCAPE):
      self.stop()

if __name__=="__main__":
  from ckbot import nobus
  app =Example(arch=nobus)
  app.run()
