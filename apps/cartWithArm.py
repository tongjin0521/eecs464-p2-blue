#!/usr/bin/env python
from joy import *

NAMES = {
  0xB : 'LT',
  0x9 : 'RT',
#  0xD : 'Z',
  0x21 : 'R0',
  0x8 : 'R1'
}

class Cart( Plan ):
  def __init__(self, app, *args, **kw):
    Plan.__init__(self,app,*args,**kw)
    self.torque = 0
    self.modir = 1
    self.ofs = 0
    self.status = ''
    
  def onEvent( self, evt ):
    if evt.type==TIMEREVENT:
      lt,rt = ((self.ofs-self.torque)*self.modir
              ,(self.ofs+self.torque)*self.modir)
      self.LT(lt)
      self.RT(rt)
      self.status = 'speed %3.1g %s turn %3.1g' % (
         self.torque, ['stop',' GO '][self.modir!=0], self.ofs )
      return False
    if evt.type!=MIDIEVENT or not evt.index in [0,2]:
      progress("OTHER evt "+describeEvt(evt))
      return False
    if evt.kind == 'slider':
      self.torque = (evt.value - 63.5)/ 256.0
      if abs(self.torque)<0.1:
        self.torque = 0
      progress("torque =  %g" % self.torque)
      return False
    elif evt.kind == 'knob':
      self.ofs = (evt.value - 63.5) / 256.0
      if abs(self.ofs)<0.1:
        self.ofs = 0
      progress("ofs =  %g" % self.ofs)
      return False
    elif evt.kind=='play':
      progress("(say) Movement is allowed ")
      self.modir = 1
    elif evt.kind=='stop':
      progress("(say) Movement stopped")
      self.modir = 0
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
    self.coarse = 0
    self.last = None
    self.tau = 1.0
    self.maxJump = 1000

  @classmethod
  def ofModule(cls,name,module):
    """
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
    """
    return MidiDOF( name, module.get_pos, module.set_pos, module.go_slack )

  def getStatus( self, width=40 ):
    s = ['-']*width
    def pos2idx(p):
      return int(width*(p/2e4+0.5))
    s[pos2idx(self.coarse)] = '+'
    p = pos2idx( self.pos )
    if p<0: 
	s[0] = "<"
    elif p>=width:
        s[-1] = ">"
    else:
      if not self.active:
        s[p] = "v"
      elif self.smooth:
        s[p] = 'S'
      else:
        s[p] = "|"
    return "".join(s)

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
        self.coarse = self.get() - self.fine
        progress("(say) %s going fast" % self.name)
      else:
        self.smooth = True
        progress("(say) %s going smooth" % self.name)
    elif evt.kind == "knob":
      self.fine = (val-63.5)*-4*(9000/63.5)/63.5
      #progress("%s  %5d *%5d*" % (self.name,self.coarse,self.fine))
    elif evt.kind == "slider":
      self.coarse = (val-63.5)*9000/63.5
      #progress("%s *%5d* %5d " % (self.name,self.coarse,self.fine))
      
  def update( self, now ):
    if self.last is None:
      self.last = now
      return
    if not self.active:
      return
    self.pos = self.coarse + self.fine
    cur = self.get()
    if not self.smooth and abs(self.pos-cur)<self.maxJump:
      goal = self.coarse
    else:
      dt = min( 1, (now - self.last)/self.tau )
      goal = cur * (1-dt) + self.coarse * dt
    self.last = now
    self.set(goal+self.fine)


class DemoJoyApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,robot=dict(names=NAMES,count=len(NAMES)),cfg=dict(clockInterval=50),**kw)
    self.voltage = 666
  
  def onStart( self ):
    # Set up a StickFilter Plan
    self.cp = Cart(self,LT="LT/@set_torque",RT="RT/@set_torque")
    self.cp.start()
    #self.Z = MidiDOF.ofModule("pivot",self.robot.at.Z)
    self.R0 = MidiDOF.ofModule("shoulder",self.robot.at.R0)
    self.R1 = MidiDOF.ofModule("elbow",self.robot.at.R1)
    self.midis = {
      0 : self.cp.push,
      2 : self.cp.push,
     # 7 : self.Z.onMidiEvent,
      8 : self.R0.onMidiEvent,
      9 : self.R1.onMidiEvent,
    }
    self.voltageTest = self.onceEvery(5)
    self.display = self.onceEvery(0.5)
    self.smoothStep = self.onceEvery(0.05)

  def onEvent( self, evt ):
    if self.voltageTest():
      self.voltage = min([ m.get_voltage() for m in self.robot.itermodules()])
      if self.voltage<15:
        if self.voltage<14:
          progress('(say) DANGER: battery voltage is %d. Quitting!' % self.voltage )
          self.stop()
        else:
          progress('(say) WARNING: battery voltage is %d. Replace or recharge')
    if self.display():
      progress(self.cp.status)
      #progress("%-10s" % self.Z.name + self.Z.getStatus())
      progress("%-10s" % self.R0.name + self.R0.getStatus())
      progress("%-10s" % self.R1.name + self.R1.getStatus())
    if self.smoothStep():
      self.R0.update(self.now)
      self.R1.update(self.now)
      #self.Z.update(self.now)
    if evt.type==MIDIEVENT:
      if evt.sc != 4 and evt.index != 0:
        progress('WARNING: must use scene 4')
        progress(describeEvt(evt))          
        return
      # dispatch to appropriate midi event handler
      try:
        h = self.midis[evt.index]
      except KeyError:
        if self.DEBUG:
          h = lambda evt : progress(describeEvt(evt)) 
        else:
          h = lambda x : None
      h(evt)
      return
    elif evt.type in [MOUSEMOTION]:
      return
    return JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  app = DemoJoyApp()
  app.run()
