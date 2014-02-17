from joy import *
from math import copysign,cos,sin,pi,degrees,abs
from numpy import matrix,floor
from numpy import linalg
from time import time as now
import ckbot.dynamixel as DX
import ckbot.logical as logical
import ckbot.nobus as NB
import math

#pull in contact gait module and turn in place module
from centipedeContact import *
import centipedeTurnInPlace as centTIP

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
    #progress("Commanded Roll " + str(self)+" " + str(pos[2]))
    if self.f!=None:
      self.f.set_pos(pos[0])
    if self.b!=None:
      self.b.set_pos(pos[1])
    if self.l!=None:
      self.l.set_pos(pos[2])


class FunctionCyclePlanApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self, confPath="$/cfg/JoyAppCentipedeV2.yml", *arg,**kw)
    self.turnInPlaceMode = 0
    self.backward = 0
    self.gaitSpec = None
    
    self.last = now()

  def fun( self, phase ):
  		  
    bend = 0	# no longer used - change bend w/ controller
    g1 = self.gait1
    g2 = self.gait2
    g3 = self.gait3
    
    #deal with forward/backward here?  Maybe with joystick? and bend?
    phi = 1 - phase              
        
	#for legs 1 & 3
    g1.manageGait(phi)
    s1roll = degrees(g1.roll)
    s1yaw = degrees(g1.yaw)
    
    g3.manageGait(phi)
    s3roll = degrees(g3.roll)
    s3yaw = degrees(g3.yaw)
	
	#for leg 2: give phi2 = (phi1 + 0.5) mod 1
    if(phi > 0.5):
        phi2 = phi - 0.5
    else:
        phi2 = phi + 0.5
        
    g2.manageGait(phi2) 
    s2roll = degrees(g2.roll)
    s2yaw = degrees(g2.yaw)
    
    if(0.5 < phi2 <= 1):	#transform for second half of gait
        s2roll = -s2roll
        s2yaw = -s2yaw
    
    if(0.5 < phi <= 1):	#transform for second half of gait
        s1roll = -s1roll
        s1yaw = -s1yaw
        s3roll = -s3roll
        s3yaw = -s3yaw
    
    #turn in place
    self.tipGait1.manageGait(phi)    
    self.tipGait2.manageGait(phi2)
    self.tipGait3.manageGait(phi)
    
    if(self.turnInPlaceMode == 1):
      #get turn-in-place gait values
      s1roll = 0#degrees(self.tipGait1.roll)
      s1yaw = 0#degrees(self.tipGait1.yaw)
      s2roll = degrees(self.tipGait2.roll)
      s2yaw = -degrees(self.tipGait2.yaw)
      s3roll = 0#degrees(self.tipGait3.roll)
      s3yaw = 0#degrees(self.tipGait3.yaw)
      
      #progress("TURNINPLACE, " + str(s1roll) + ", " + str(s1yaw))
  
    if(self.backward == 1): #transform if moving backwards (invert roll)
      s1roll = -s1roll
      s2roll = -s2roll
      s3roll = -s3roll       
    else:
      pass
            
    #roll/yaw are deg, need to convert to centidegrees
    self.S1.set_pos(s1yaw*100, g1.bend, s1roll*100)  
    self.S2.set_pos(s2yaw*100, g2.bend, s2roll*100)   
    self.S3.set_pos(s3yaw*100, g3.bend, -s3roll*100)      #facing opposite     

  def onStart(self):
    self.S1 = Segment(None, self.robot.at.B1, self.robot.at.L1)
    self.S2 = Segment(self.robot.at.F2, self.robot.at.B2, self.robot.at.L2)
    self.S3 = Segment(self.robot.at.F3, None, self.robot.at.L3)
    
    
      #set the slope parameter for each motor:
    for m in self.robot.itermodules():
      m.mem[m.mcu.ccw_compliance_slope] = 64
      m.mem[m.mcu.cw_compliance_slope] = 64

    self.last = 0
    self.backward = 0
    
    #setup parameters for contact gait - some reasonable values
    initParams = gaitParams()
    initParams.rollThresh = -(29*pi/180)
    initParams.yawThresh = -(9*pi/180)
    initParams.maxRoll = (34*pi/180)
    initParams.rollAmp = (34*pi/180)
    initParams.yawAmp = (10*pi/180)
    initParams.stanceVel = 1.26
    
    self.gait1 = contactGait(initParams, self.S1.l, 2322)
    self.gait2 = contactGait(initParams, self.S2.l, 0)
    self.gait3 = contactGait(initParams, self.S3.l, 0)
    
    tipParamsFB = gaitParams()  # rollAmp and yawAmp are only params used for TIP
    tipParamsFB.rollAmp = (33*pi/180)
    tipParamsFB.yawThresh = -(40*pi/180)  
    tipParamsFB.yawAmp = (10*pi/180)
    
    tipParamsMid = gaitParams()  # rollAmp and yawAmp are only params used for TIP
    tipParamsMid.rollAmp = (33*pi/180)
    tipParamsMid.yawThresh = -(40*pi/180)  
    tipParamsMid.yawAmp = (10*pi/180)
       
    
    self.tipGait1 = centTIP.turnInPlaceGait(tipParamsFB)
    self.tipGait2 = centTIP.turnInPlaceGait(tipParamsMid)
    self.tipGait3 = centTIP.turnInPlaceGait(tipParamsFB)

    self.gaitSpec = Struct(
      rollAmp = 0.4, 
      yawAmp = 0,
      phi_s = 0.94,
      t_s = 0.23,
      turn = 0,
      freq = 1,
      ecc =0,

      maxFreq = 3.0,   
      maxYaw = 700,
      maxRoll = 7000,
      maxTurn = -500,
      maxEcc = math.pi/2,
    )

    self.plan = FunctionCyclePlan(self, self.fun,N=302,interval=0.02)
    self.plan.onStart = curry(progress,">>> START")
    self.plan.onStop = curry(progress,">>> STOP")
    self.plan.setFrequency(self.gaitSpec.freq)
    self.dir = 1
    
    sf = StickFilter(self)
    sf.setLowpass("joy0axis0",5)
    sf.setLowpass("joy0axis1",5)
    sf.start()
    self.sf = sf
    
    self.timeToShow = self.onceEvery(0.25)
    self.timeToPlot = self.onceEvery(0.5)

  def onStop(self):
    self.robot.off()
  
  def onEvent(self, evt):
    gs = self.gaitSpec

    #if self.timeToPlot():
    #  progress('time: %g' % (time.time()))

    if self.timeToShow():
      gs.turn = 0.5*self.sf.getValue("joy0axis2")
      gs.yawAmp = -0.5#self.sf.getValue("joy0axis3")
      progress('freq: %g, bend: %g, backwards: %g, strafe: %g, turn mode: %g' 
               % (gs.freq, self.gait1.bend, self.backward, self.gait1.strafe, self.turnInPlaceMode))

    if evt.type==JOYBUTTONDOWN and evt.joy==0:
      progress( describeEvt(evt) )
      # start
      if evt.button==5:
        self.plan.start()
        progress('--> starting plan')
      # stop
      if evt.button==7:
        self.robot.off()
        self.plan.stop()
        progress('STOP')
      if evt.button==2: #increase freq
        gs.freq += 0.1
        self.plan.setFrequency( gs.freq )
      if evt.button==0: #decrease freq
        gs.freq -= 0.1
        self.plan.setFrequency( gs.freq )        
      if evt.button==3: #increase bend
        #gs.rollAmp += 0.01
        g1 = self.gait1
        g2 = self.gait2
        g3 = self.gait3
        
        #bend in centidegrees
        g1.bend += 100
        g2.bend += 100
        g3.bend += 100
      if evt.button==1: #decrease bend
        #gs.rollAmp -= 0.01
        g1 = self.gait1
        g2 = self.gait2
        g3 = self.gait3
        
        g1.bend -= 100
        g2.bend -= 100
        g3.bend -= 100
      if evt.button==6: #turn in place mode = 1
        self.turnInPlaceMode = 1
      if evt.button==4: #turn in place mode = 0
        self.turnInPlaceMode = 0
      if evt.button==8: #increase strafing
        self.gait1.strafe += 10
        self.gait2.strafe += 10
        self.gait3.strafe += 10
      if evt.button==9: #decrease strafing
        self.gait1.strafe -= 10
        self.gait2.strafe -= 10
        self.gait3.strafe -= 10
      if evt.button==10: #backward = 1
        self.backward = 1
      if evt.button==11: #backward = 0
        self.backward = 0
      return
    if evt.type==KEYDOWN:
      if evt.key==ord('m'):#
          self.plan.start()#
      if evt.key==ord('e'): # 'e' increases eccentricity
          gs.ecc += 0.1
      if evt.key==ord('d'): # 'd' decreases eccentricity
          gs.ecc -= 0.1

    if evt.type in [JOYAXISMOTION]:
      self.sf.push(evt)
      #put joystick controls in here
      #can get the value for freq and bend with:
      #self.sf.getValue("joy0axis0") = freq, self.sf.getValue("joy0axis1")=bend
      #then remove the button stuff? not necessary, but would be cleaner
      
      #update gait with joystick inputs
      bend = self.sf.getValue("joy0axis1") #value range: [-1, 1]
      freq = self.sf.getValue("joy0axis0") #value range: [-1, 1]
      
      maxFreq = 2
      maxBend = 1500 #centi-degrees
      
      #handle negative frequency
      if(freq < 0):
        freq = abs(freq)
        self.backward = 1
      else:
        self.backward = 0
        
      self.plan.setFrequency(maxFreq*freq)
      self.gait1.bend = (maxBend*bend)
      self.gait2.bend = (maxBend*bend)
      self.gait3.bend = (maxBend*bend)
      
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
  
  # for actual operation use w/ arch=DX & NO required line:  
  ckbot.logical.DEFAULT_BUS = ckbot.dynamixel
  ckbot.logical.DEFAULT_PORT = dict(TYPE="tty", baudrate=1000000, glob="/dev/ttyUSB*")
  
  # for simulation use w/ arch=NB & required line:
  #ckbot.logical.DEFAULT_BUS = ckbot.nobus
  
  app=FunctionCyclePlanApp(robot=dict(
    arch=DX,count=7,fillMissing=True,
    #required=[ 0x26, 0x08, 0x06, 0x2D, 0x31, 0x27, 0x22 ] 
    ))
  
  app.run()

