#!/usr/bin/env python
from joy import *

NAMES = {
  0xC : 'LT',
  0x2 : 'RT',
  0xD : 'Z',
  0x3 : 'R0',
  0x4 : 'R1'
}

class Cart( Plan ):
  def __init__(self, app, *args, **kw):
    Plan.__init__(self,app,*args,**kw)
    self.torque = 0
    self.modir = 1
    self.ofs = 0
    self.timeToShow = app.onceEvery(0.5)
    
  def onEvent( self, evt ):
    if evt.type==TIMEREVENT:
      if self.timeToShow():
        progress("Set LT %.2g RT %.2g" 
        %  ((self.ofs-self.torque)*self.modir
            ,(self.ofs+self.torque)*self.modir)
        + "  %g %g %g" % (self.torque,self.modir,self.ofs)
        )
      return False
    if evt.type!=MIDIEVENT or not evt.index in [0,2]:
      progress("OTHER evt "+describeEvt(evt))
      return False
    if evt.kind == 'slider':
      self.torque = self.modir * evt.value / 256.0
      if abs(self.torque)<0.1:
        self.torque = 0
      progress("torque =  %g" % self.torque)
      return False
    elif evt.kind == 'knob':
      self.ofs = (evt.value - 63.5) / 127
      if abs(self.ofs)<0.1:
        self.ofs = 0
      progress("ofs =  %g" % self.ofs)
      return False
    elif evt.kind == "btnL":
      if not self.modir <= 0:
        progress("release other direction first")
      elif evt.value:
        self.modir = -1
        progress("(say) Direction is reverse")
      else:
        self.modir = 0
        progress("(say) Reverse motion stopped")
    elif evt.kind == "btnU":
      if not self.modir >= 0:
        progress("release other direction first")
      elif evt.value:
        self.modir = 1
        progress("(say) Direction is forward")
      else:
        self.modir = 0
        progress("(say) Forward motion stopped")
    elif evt.kind=='play':
      progress("(say) Move ")
    elif evt.kind=='stop':
      progress("(say) Stop")
    else:
      progress("OTHER 127 "+describeEvt(evt))
    return False      

class MidiDOF( object ):
  def __init__(self, name, getter, setter, slacker):
    self.name = name
    self.get = getter
    self.set = setter
    self.slack = slacker
    self.active = False
    self.smooth = False
    self.pos = self.get()
    self.fine = 0
    self.last = None
    self.tau = 0.5
  
  @classmethod
  def ofModule(cls,name,module):
    x = [0]
    def g():
      progress("!!! get --> %g" % x[0])
      return x[0]
    def s(v):
      progress("!!! set --> %g" % v)
      x[0] = v
    def k():
      progress("!!! slack")
    return MidiDOF( name, g, s, k )
    return MidiDOF( name, module.get_pos, module.set_pos, module.go_slack )
    
  def onMidiEvent( self, evt ):
    val = evt.value
    if evt.kind == "btnU":
      if val>0:
        self.slack()
        self.active = False
        progress("(say) %s going slack" % self.name)
      else:
        self.active = True
        progress("(say) %s activated" % self.name)
    elif evt.kind == "btnL":
      if val>0:
        self.smooth = False
        progress("(say) %s going fast" % self.name)
      else:
        self.smooth = True
        progress("(say) %s going smooth" % self.name)
    elif evt.kind == "knob":
      self.fine = (val-63.5)*2*(9000/63.5)/63.5
    elif evt.kind == "slider":
      self.coarse = (val-63.5)*9000/63.5
      
  def update( self, now ):
    if self.last is None:
      self.last = now
      return
    if not self.active:
      return
    self.pos = self.coarse + self.fine
    if not self.smooth:
      goal = self.pos
    else:
      dt = min( 1, (now - self.last)/self.tau )
      cur = self.get() 
      goal = cur * (1-dt) + self.pos * dt
    self.last = now
    self.set(goal)

class DemoJoyApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
  
  def onStart( self ):
    # Set up a StickFilter Plan
    self.cp = Cart(self)
    self.cp.start()
    self.Z = MidiDOF.ofModule("pivot",None)#self.robot.at.Z)
    self.R0 = MidiDOF.ofModule("shoulder",None)#self.robot.at.R0)
    self.R1 = MidiDOF.ofModule("elbow",None)#self.robot.at.R1)
    self.midis = {
      0 : self.cp.push,
      2 : self.cp.push,
      7 : self.Z.onMidiEvent,
      8 : self.R0.onMidiEvent,
      9 : self.R1.onMidiEvent,
    }
    self.warnOnce = self.onceEvery(5)

  def onEvent( self, evt ):
    if evt.type==MIDIEVENT:
      if evt.sc != 4:
        if self.warnOnce():
          progress('(say) must use scene 4')
        progress(describeEvt(evt))          
        return
      # dispatch to appropriate midi event handler
      try:
        h = self.midis[evt.index]
      except KeyError:
        h = lambda evt : progress(describeEvt(evt)) 
      h(evt)
    elif evt.type == TIMEREVENT:
      self.R0.update(self.now)
      self.R1.update(self.now)
      self.Z.update(self.now)
    elif evt.type in [MOUSEMOTION]:
      return
    return JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  app = DemoJoyApp()
  app.run()
