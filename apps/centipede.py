from joy import *
from math import copysign,cos,sin,pi
from numpy import matrix,floor
from numpy import linalg
import time
import ckbot.hitec as hitec
import ckbot.logical as logical
import math

# For plotting Buehler
import matplotlib
matplotlib.use('TkAgg') #Uses a backend that is usable in OSX as well
import matplotlib.pyplot as plt
import pylab
import numpy as np
"""
WARNING

I refactored this to work with the flipnworm.py app.  FunctionCyclePlanApp needs to be fixed.

"""
def dblStance( t, phi_s, t_s, phi_o, t_o, bend = 0):
    """
    Implements Buehler clock and returns phase as a function of time. It has 
    two slow phases.

    INPUTS:
      t -- time -- between -0.5 to 0.5 
      phi_s -- angle swept during stance
      t_s -- total stance time of one leg
      phi_o -- stance angle offset
      t_o -- stance time offset
      bend -- radians -- bend angle of the body

    OUTPUT:
      res -- phase -- from -pi to pi
    """
    t += t_o
    t -= floor(t)
    
    flag = 1
    if t >= 0.5:
      t = 1.0-t
      flag = -1

    assert phi_s >= 0 and phi_s <= math.pi
    assert t_s >= 0 and t_s <= 0.5

    #calculate relative stance angles
    M = 60
    L = 150
    if abs(bend) > 0.1:
      r = (M/2 + M*math.cos(bend))/math.sin(bend)
      phi_sb = r/(r-L)*phi_s
      phi_st = r/(r+L)*phi_s
    else:
      phi_sb = phi_s
      phi_st = phi_s

    #calculate slopes
    w_sb = phi_sb/t_s
    w_st = phi_st/t_s
    wf = (2.0*math.pi - phi_sb - phi_st)/(1.0-2*t_s)

    if abs(t)<=(t_s/2):
      res = w_sb*t
    elif abs(t)>(t_s/2) and abs(t) <= (0.5 - t_s/2):
      res = wf*(t-t_s/2)+phi_sb/2
    elif abs(t)>(0.5 - t_s/2):
      res = w_st*(t-(0.5-t_s/2)) + (math.pi-phi_st/2)

    #if res < -math.pi:
    #  res = res + 2*math.pi
    return res*flag + phi_o   

def dblStanceIterable(time,phi_sb,phi_st,t_s,phi_o,t_o):
    assert np.iterable(time)
    res = []
    for t in time:
        res.append(dblStance(t,phi_sb,phi_st,t_s,phi_o,t_o))
    return np.asarray(res)

class Struct(object):
  def __init__(self,**kw):
    self.__dict__.update(kw)

class Segment(object):
  def __init__(self, front, back, leg):
    self.f = front
    self.b = back
    self.l = leg
        
  def set_pos(self,yaw,bend,roll):
    A = matrix([[2,0.5,0],[-2,0.5,0],[0,0,1]],dtype='float')
    pos=A*matrix([[yaw],[bend],[roll]])
    if self.f!=None:
      self.f.set_pos(pos[0])
    if self.b!=None:
      self.b.set_pos(pos[1])
    if self.l!=None:
      self.l.set_pos(pos[2])


class FunctionCyclePlanApp( JoyApp ):
  def __init__(self,*arg,**kw):

    self.bus = hitec.Bus()
    # set up protocol with hitec bus    
    p = hitec.Protocol(bus = self.bus)
    kw.update(robot=dict(protocol = p, count=7))
    JoyApp.__init__(self, confPath="$/cfg/JoyAppCentipede.yml", *arg,**kw)
    self.turnInPlaceMode = 0
    self.gaitSpec = None
    self.last = time.time()

  def fun( self, phase ):
    g = self.gaitSpec
    phase1 = dblStance(phase, g.phi_s, g.t_s, 0,0, -g.turn)
    phase2 = dblStance(phase, g.phi_s, g.t_s, 0,0.5, -g.turn)
    bend = 0 #-turn*maxTurn
    roll1 = math.cos(phase1)*g.maxRoll*g.rollAmp

    if self.turnInPlaceMode: # create figure 8
      yaw1 = -math.sin(phase1*2)*1.3*g.maxYaw*g.yawAmp 
      #1.3 b/c can make longer strides
    else:
      yaw1 = -math.sin(phase1+g.yawAmp*g.ecc)*g.maxYaw*g.yawAmp

    roll2 = math.cos(phase2)*g.maxRoll*g.rollAmp
    if self.turnInPlaceMode:
      yaw2 = -math.sin(phase1*2)*1.3*g.maxYaw*g.yawAmp #turning
    else:
      yaw2 = -math.sin(phase2)*g.maxYaw*g.yawAmp #turning

    self.S1.set_pos(yaw1,bend,roll1)
    self.S2.set_pos(yaw2,bend,roll2)
    self.S3.set_pos(yaw1,bend,roll1)

  def onStart(self):
    self.S1 = Segment(None, self.robot.at.B1, self.robot.at.L1)
    self.S2 = Segment(self.robot.at.F2, self.robot.at.B2, self.robot.at.L2)
    self.S3 = Segment(self.robot.at.F3, None, self.robot.at.L3)

    self.last = 0

    self.gaitSpec = Struct(
      rollAmp = 0.4, 
      yawAmp = 0,
      phi_s = 0.94,
      t_s = 0.23,
      turn = 0,
      freq = 2,
      ecc =0,

      maxFreq = 3.0,   
      maxYaw = 700,
      maxRoll = 7000,
      maxTurn = -500,
      maxEcc = math.pi/2,
    )

    self.plan = FunctionCyclePlan(self, self.fun,N=180,interval=0.05)
    self.plan.onStart = curry(progress,">>> START")
    self.plan.onStop = curry(progress,">>> STOP")
    self.plan.setFrequency(self.gaitSpec.freq)
    self.dir = 1

    sf = StickFilter(self)
    sf.setLowpass("joy0axis2",10)
    sf.setLowpass("joy0axis3",10)
    sf.start()
    self.sf = sf
    self.timeToShow = self.onceEvery(0.25)
    self.timeToPlot = self.onceEvery(0.5)

  def onStop(self):
    self.robot.off()
  
  def onEvent(self, evt):
    gs = self.gaitSpec

    if self.timeToPlot():
      progress('time: %g' % (time.time()))

    if self.timeToShow():
      gs.turn = 0.5*self.sf.getValue("joy0axis2")
      gs.yawAmp = -0.5#self.sf.getValue("joy0axis3")
      progress('freq: %g, roll: %g, yaw: %g, turn: %g, turn mode: %g' 
               % (gs.freq, gs.rollAmp, gs.yawAmp, gs.turn, self.turnInPlaceMode))

    if evt.type==JOYBUTTONDOWN and evt.joy==0:
      progress( describeEvt(evt) )
      # start
      if evt.button==5:
        self.plan.start()
        progress('--> starting plan')
      # stop
      if evt.button==7:
        self.plan.stop()
        self.robot.off()
        progress('STOP')
      if evt.button==2: #increase freq
        gs.freq += 0.1
        self.plan.setFrequency( gs.freq )
      if evt.button==0: #decrease freq
        gs.freq -= 0.1
        self.plan.setFrequency( gs.freq )        
      if evt.button==3: #increase roll
        gs.rollAmp += 0.01
      if evt.button==1: #decrease roll
        gs.rollAmp -= 0.01
      if evt.button==6: #turn in place mode = 1
        self.turnInPlaceMode = 1
      if evt.button==4: #turn in place mode = 0
        self.turnInPlaceMode = 0
      return
    if evt.type==KEYDOWN:
      if evt.key==ord('e'): # 'e' increases eccentricity
          gs.ecc += 0.1
      if evt.key==ord('d'): # 'd' decreases eccentricity
          gs.ecc -= 0.1

    if evt.type in [JOYAXISMOTION]:
      self.sf.push(evt)
      return
    if evt.type!=TIMEREVENT:
      JoyApp.onEvent(self,evt)

class GaitTestPlanApp( JoyApp ):

# THIS IS FOR TESTING WITHOUT BEING CONNECTED TO ROBOT

  def __init__(self,*arg,**kw):
    JoyApp.__init__(self, confPath="$/cfg/JoyAppCentipede.yml", *arg,**kw)
    self.turnInPlaceMode = 0
    self.gaitSpec = None
    self.last = time.time()
    self.t = np.arange(-0.5,0.5,0.05)
    phi = dblStanceIterable(self.t,0.94,0.23,0,0,0)
    self.h, = pylab.plot(self.t,phi)

  def fun( self, phase ):
    g = self.gaitSpec
    phase1 = dblStance(phase, g.phi_s, g.t_s, 0,0, -g.turn)
    phase2 = dblStance(phase, g.phi_s, g.t_s, 0,0.5, -g.turn)
    bend = 0 #-turn*maxTurn
    roll1 = math.cos(phase1)*g.maxRoll*g.rollAmp

    if self.turnInPlaceMode: # create figure 8
      yaw1 = -math.sin(phase1*2)*1.3*g.maxYaw*g.yawAmp 
      #1.3 b/c can make longer strides
    else:
      yaw1 = -math.sin(phase1+g.yawAmp*g.ecc)*g.maxYaw*g.yawAmp

    roll2 = math.cos(phase2)*g.maxRoll*g.rollAmp
    if self.turnInPlaceMode:
      yaw2 = -math.sin(phase1*2)*1.3*g.maxYaw*g.yawAmp #turning
    else:
      yaw2 = -math.sin(phase2)*g.maxYaw*g.yawAmp #turning

  def onStart(self):
    self.last = 0

    self.gaitSpec = Struct(
      rollAmp = 0.2, 
      yawAmp = 0,
      phi_s = 0.94,
      t_s = 0.23,
      turn = 0,
      freq = 2,
      ecc =0,

      maxFreq = 3.0,   
      maxYaw = 700,
      maxRoll = 7000,
      maxTurn = -500,
      maxEcc = math.pi/2,
    )

    self.plan = FunctionCyclePlan(self, self.fun,N=180,interval=0.05)
    self.plan.onStart = curry(progress,">>> START")
    self.plan.onStop = curry(progress,">>> STOP")
    self.plan.setFrequency(self.gaitSpec.freq)
    self.dir = 1

    sf = StickFilter(self)
    sf.setLowpass("joy0axis2",10)
    sf.setLowpass("joy0axis3",10)
    sf.start()
    self.sf = sf
    self.timeToShow = self.onceEvery(1)

  def onStop(self):
    progress('Stopped')
  
  def onEvent(self, evt):
    gs = self.gaitSpec

    if self.timeToShow():
      gs.turn = 0.5*self.sf.getValue("joy0axis2")
      gs.yawAmp = -self.sf.getValue("joy0axis3")
      progress('freq: %g, roll: %g, yaw: %g, turn: %g, turn mode: %g' 
               % (gs.freq, gs.rollAmp, gs.yawAmp, gs.turn, self.turnInPlaceMode))
      phi = dblStanceIterable(self.t,gs.phi_s,gs.t_s,0,0,-gs.turn)
      self.h.set_ydata(phi)
      pylab.draw()

    if evt.type==JOYBUTTONDOWN and evt.joy==0:
      progress( describeEvt(evt) )
      # plot
#      if evt.button==6:
      # start
      if evt.button==5:
        self.plan.start()
        progress('--> starting plan')
      # stop
      if evt.button==7:
        self.plan.stop()
        progress('STOP')
      if evt.button==2: #increase freq
        gs.freq += 0.1
        self.plan.setFrequency( gs.freq )
      if evt.button==0: #decrease freq
        gs.freq -= 0.1
        self.plan.setFrequency( gs.freq )        
      if evt.button==3: #increase roll
        gs.rollAmp += 0.01
      if evt.button==1: #decrease roll
        gs.rollAmp -= 0.01
      if evt.button==6: #turn in place mode = 1
        self.turnInPlaceMode = 1
      if evt.button==4: #turn in place mode = 0
        self.turnInPlaceMode = 0
      return
    if evt.type==KEYDOWN:
      if evt.key==ord('e'): # 'e' increases eccentricity
          gs.ecc += 0.1
      if evt.key==ord('d'): # 'd' decreases eccentricity
          gs.ecc -= 0.1

    if evt.type in [JOYAXISMOTION]:
      self.sf.push(evt)
      return
    if evt.type!=TIMEREVENT:
      JoyApp.onEvent(self,evt)



if __name__=="__main__":
  print """
  Centipede
  -------------------------------
  
  When any key is pressed, a 7-module centipede commences locomotion.

  The application can be terminated with 'q' or [esc]
  """
  robot = None
  
 # import cProfile
  import joy
  joy.DEBUG[:]=[]
  app=FunctionCyclePlanApp(robot=dict(count=7))
#  app=GaitTestPlanApp()
  app.run()
#  cProfile.run(  app.run()  , 'centipede_profile_stats')

