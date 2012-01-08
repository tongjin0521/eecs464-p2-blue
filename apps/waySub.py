from joy import *
from numpy import *
from pylab import *
from os import system
from time import time as now
T0 = now()

system("pkill espeak")
try:
  app.remote.sock.close()
  progress("recyling socket....")
except NameError:
  pass
    
class Turn( Plan ):
  def __init__(self,*argv,**kwarg):
    Plan.__init__(self,*argv,**kwarg)
    self.brCoef = 0.6
    self.ang = 0
    self.torque = 0.2
    self.duration = 0.1
  
  def setTurn(self,ang = None,torque=0.2):
    if self.isRunning():
      progress("WARNING: cannot call setTurn() while self.isRunning()")
    else:
      if ang is not None:
        self.ang = float(ang)
      self.torque = torque

  def behavior( self ):
    last = self.app.now
    tq = self.torque*sign(self.ang)
    dur = self.duration * abs(self.ang)
    lm = self.app.robot.at.LT
    rm = self.app.robot.at.RT
    tm = self.app.robot.at.TT
    #
    #progress("Starting Turn(%d)" % self.ang)
    lm.set_torque(tq)
    rm.set_torque(tq)
    tm.set_torque(tq*self.brCoef)
    yield self.forDuration(dur)
    lm.go_slack()
    rm.go_slack()
    tm.go_slack()
    #progress("Ending Turn(%d)" % self.ang)

class Move( Plan ):
  def __init__(self,*argv,**kwarg):
    Plan.__init__(self,*argv,**kwarg)
    self.distance = 0
    self.torque = 0.2
    self.duration = 0.1
    self.ass = -0.05
    
  def set_distance(self,distance):
    if self.isRunning():
      progress("WARNING: cannot call set_distance() while self.isRunning()")
    else:
      self.distance = float(distance)

  def behavior( self ):
    last = self.app.now
    tq = self.torque*sign(self.distance)
    dur = self.duration * abs(self.distance)
    lm = self.app.robot.at.LT
    rm = self.app.robot.at.RT
    tm = self.app.robot.at.TT
     #
    #progress("Starting Move(%d)" % self.distance)
    lm.set_torque(tq*(self.ass+1))
    rm.set_torque(-tq)
    yield self.forDuration(dur)
    lm.go_slack()
    rm.go_slack()
    #progress("Ending Move(%d)" % self.distance)
  
class SenseMixin( object ):
  def __init__(self):
    self.sens = []
    self.way = None

  def getSense( self, N=2 ):
    while True:
      for t,dic in self.app.remote.queueIter():
        self.sens.append((t-T0,dic['f'],dic['b']))
        if dic.has_key('w'):
          self.way = dic['w']
      if len(self.sens)>=N:
        break
      yield self.forDuration(0.3)

class CastForLine( Plan, SenseMixin ):
  def __init__(self,app,move,turn,*argv,**kwarg):
    Plan.__init__(self,app,*argv,**kwarg)
    SenseMixin.__init__(self)
    self.move = move
    self.turn = turn
    self.thr = 200
    self.far = 60
    self.lost = 20
    self.tic = 10
  
  def behavior(self):
    q = self.app.remote
    # Allow sensor data in
    q.setAllowMisc(20)
    tic = self.tic
    while True:
      self.sens = []
      q.flushMisc()
      yield self.getSense(8)
      s = asarray(self.sens)
      print "!!!@@",s
      mf = mean(s[:,1]) 
      if mf > self.thr:
        progress("(say) found it!")
        return
      if mf < self.far:
        if mf < self.lost:
          progress("(say) casting lost the line")
          return          
        progress("(say) oh no! I've gone too far")
        tic = -tic
      self.turn.setTurn(tic,torque=0.13)
      yield self.turn
      
class LineFollow( Plan, SenseMixin ):
  def __init__(self,app,move,turn,cast,*argv,**kwarg):
    Plan.__init__(self,app,*argv,**kwarg)
    SenseMixin.__init__(self)
    self.move = move
    self.turn = turn
    self.cast = cast
    
  def behavior( self ):
    # No previous sensor measurements to include
    self.sens = []
    w0 = None
    w = None
    q = self.app.remote
    # Allow sensor data in
    q.setAllowMisc(20)
    q.flushMisc()
   
    while True:
      yield self.getSense()
      # ASSERT: we have at least two sensor readings
      # If we got waypoint information 
      if self.way:
        # Convert to complex form
        w = self.way[0][0]+self.way[0][1]*1j
        # If first time --> remember it
        if w0 is None:
          w0 = w
        elif abs(w0-w)>10: # else if moved by 'enough' 
          # --> done with this line
          break
      s = asarray(self.sens)
      self.sens[:] = []
      st,sf,sb = s.T
      print "!!!",s
      self.move.set_distance(10)
      yield self.move
      # If all four sensor readings are high --> keep moving
      if all(sf>230):
        continue
      # If all back are better than all front --> move back
      if all(sf<sb-20):
        self.move.set_distance(-5)
        yield self.move
        yield self.cast
        continue        
      # If all four are small --> lost the line
      if all(sf<30):
        progress("(say) lost the line")
        break
      # Compare before and after
      n = s.shape[0]/2
      t0,f0,b0 = median(s[:n,...],0)
      t1,f1,b1 = median(s[n:,...],0)
      # If we are moving further away --> time to reorient
      if f1 < 200 and f1 < f0 - 20:
        yield self.cast
            
    print "!!!w0",w0
    print "!!!w", w
    
class WaySubApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__( self, confPath="$/cfg/waySubApp.yml",
      robot=dict(
        count=3,
        names={0x2:'LT',0x6:'TT',0xC:'RT'},
      ),
      cfg = dict( 
        remote = dict( 
          allowMisc = 10,
          bnd = ("0.0.0.0",0xBAA),
    ) ) )
  
  def onStart(self):
    self.turn = Turn(self)
    self.move = Move(self)
    self.cast = CastForLine(self,self.move, self.turn)
    self.linefollow = LineFollow(self,self.move, self.turn, self.cast)
    self.beh = None
    self.voltChk = self.onceEvery(5)

  def onStop( self ):
    # Make sure we go_slack on all servo modules when we exit
    self.robot.off()
  
  def onEvent(self, evt):
    if evt.type==QUIT or (evt.type==KEYDOWN and evt.key in [K_q,K_ESCAPE]):
      self.stop()
    # Check voltage periodically
    if self.voltChk():
      v = min( 
        self.robot.at.LT.get_voltage(),
        self.robot.at.RT.get_voltage(),
        self.robot.at.TT.get_voltage()
        )
      if v<13.5:
        progress("(say) Voltage is low -- replace battery at once!")
        self.stop()
      else:
        progress("Battery voltage: %d" % v)
      return
    #    
    if self.beh and not self.beh.isRunning():
      progress("Ended behavior:"+str(self.beh))
      progress("sensor fb: "+repr(list(self.remote.queueIter())))
      self.beh = None      
    # keyboard control
    if evt.type is KEYDOWN:
      if self.beh:
        if evt.key is K_s:
          progress("(say) forced stop")
          self.beh.stop()
          self.beh = None
        else:
          progress("Current behavior is:" + str(self.beh))
        return
      if evt.key is K_m:
        self.move.set_distance(10)
        self.beh = self.move
      elif evt.key is K_b:
        self.move.set_distance(-10)
        self.beh = self.move
      elif evt.key is K_j:
        self.turn.setTurn(-10)
        self.beh = self.turn
      elif evt.key is K_k:
        self.turn.setTurn(10)
        self.beh = self.turn
      elif evt.key is K_f:
        progress("(say) line following activated")
        self.beh = self.linefollow
      #
      if not self.beh:
        return
      progress("Starting behavior:" + str(self.beh))
      self.beh.start()      
    #
    # We reach this point if no previous handler processed the event.
    #   If it is not a TIMEREVENT, we use the superclass onEvent handler
    #   which prints a human readable description of the event object.
    if evt.type != MOUSEMOTION:
      JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  print """
  """
  app=WaySubApp()
  app.run()

