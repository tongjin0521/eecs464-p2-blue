import pdb
from joy import * #using joy in ckbot/trunk

from math import cos,sin,pi,degrees, atan2
import cmath
from numpy import *
import time
import ckbot.xbee
import ckbot.pololuCluster as polo
import sys
import os
import cPickle
sys.path.append('../../../vicon/python-libs')
from viconreader import ViconReader
from viconparser import ViconParser
from optutil import fmin,fminIter
import util

NOROBOT = True

if NOROBOT:
  import ckbot.nocando

class OptimClass(object):
    """
    Contains information about the robot, vicon, and general optimization parameters
    Also allows for initialization optimization variables
    """
    def __init__(self): 
        self.vp = ViconParser()
        self.vr = ViconReader()


    def initialize(self):
        # Initializes calibrates, starts up vicon
        # Prepare to stream with ViconReader
        self.vp.useInfo( self.vr.connect() )
        stream = self.vr.stream()
        self.st = self.vp.parseStream(stream) # Parse once to get size
        self.st.next() # Initial buffer
        self.lasttime=time.time()
        self.xyz=[]
        self.x=[]
        self.y=[]
        self.z=[]

    def _store_vicon_data(self):
      self.xyz.append(self.vp.xyz)
      self.x.append(self.vp.x)
      self.y.append(self.vp.y)
      self.z.append(self.vp.z)

    def _clear_vicon_data(self):
      self.xyz = []
      self.x = []
      self.y = []
      self.z = []

    def next(self, thresh = 0.004):
      """
      Keeps calling vp.next() until next() blocks for longer than thresh. It 
      takes xyz,x,y and z data from vp and stores it in OptimClass.

      INPUTS
        thresh -- threshhold for how long next() is allowed to block -- default is 0.004,
          vicon runs at around 120fps, every 8ms. st.next() should return faster than 4ms.
      """
      dt = 0
      self._clear_vicon_data()
      while dt < thresh:
        now = time.time()
        self.st.next()
        dt = time.time() - now
        self._store_vicon_data()

      
def nanMean(x,axis=None):
  """Mean of array along axis, ignoring all NaNs"""
  notNan = sum(~isnan(x),axis=axis)
  if abs(notNan) > 0.1 : #if greater than zero
    return  nansum(x,axis=axis)/notNan
  else:
    return float('Nan')


class RunAcrossPlan (Plan):
  def __init__( self, app, opto):
    Plan.__init__(self, app)
    self.opto = opto
    self.app = app
    
  def onStart( self ):
    self.app.plan.start()

  def onStop( self ):
    self.opto.next()
    self.app.xy_end = complex(nanMean(self.opto.x),nanMean(self.opto.y))
    self.app.t_end = time.time()
    self.app.plan.stop()
    self.app.c.zeros()
    self.app.c.off()
    progress("turning off robot")

  def behavior( self ):
    end_zone_upper = 500#400
    end_zone_lower = -400#-300
    slc_head = [0,8,12,3]
    slc_tail = [4,11,2,10]
    i_head = 0 
    i_tail = 10

    stop=False
    self.opto.next()
    while (not stop):
      self.opto.next()
      cPickle.dump(self.opto.xyz,self.opto.vpw)
      last_frame = self.opto.xyz[-1]

      x = nanMean(last_frame[0,:,0])
      y = nanMean(last_frame[0,:,1])
      center = complex(x,y)

      head = complex(nanMean(last_frame[0,slc_head,0]),
                     nanMean(last_frame[0,slc_head,1]))
      tail = complex(nanMean(last_frame[0,slc_tail,0]),
                     nanMean(last_frame[0,slc_tail,1]))

      target_dy = 200
      target = complex(0,y+self.app.direction*target_dy)

      if self.app.direction == 1:
        h = head-tail #heading of robot
      else:
        h = tail-head #heading of robot
      v = target-center #vector to from center of robot to goal)

      h_tar = v/h #vector pointing to goal in robot frame
      k = 1/pi
      turn = self.app.direction*k*(cmath.phase(h_tar))
      if(~isnan(turn)):
        self.app.turn = turn

      # check if reached goal  
      if self.app.direction == 1:
        if y > end_zone_upper:
          self.app.direction = -1
          stop = True
      elif self.app.direction == -1:
        if y < end_zone_lower:
          self.app.direction = 1
          stop = True

      progress("x: %0.1f, y: %0.1f,turn: %0.1f"%(x, y, turn))
      yield self.forDuration(0.25)


def logOpen(optimlogname, default = "optimlog.txt"):
    """
    Asks user for file name and whether to start a new optimization or resume
    Returns [fd,fr] - a file descriptor for writing and reading respectively
    FIXME: I couldnt get the file descriptor to be both read and write
    """
    if optimlogname == '':
        optimlogname = default
        print("Writing to '%s'\n"%default)
    m = 'a'
    if os.path.isfile(optimlogname):
        tmp = open(optimlogname,'r')

        if (len(tmp.readlines()) <= 1):
            print("Stored log file not big enough for resume! Creating new one in its place.\n")
            m = 'w'
        else:
            b = raw_input("Existing log detected! Would you like to continue the optimization [Y/n]\n").lower()
            if (b == 'n' or b == "no"):
                b = raw_input("Are you sure you want a new file? The existing log will be deleted! [y/N]\n")
                if (b == 'y' or b == "yes"):
                    m = 'w'
        tmp.close()

    return [open(optimlogname,m),open(optimlogname,'r')] # Open as read and write/append

class OptimizerApp( JoyApp ):
  """
  Higher level app to start the FunctionCyclePlan and RunAcross plans.

  FunctionCyclePlan is used to send position commands to the robot, whereas
  RunAcross reads the VICON and returns the robot's state.
  """
  def __init__(self,optobj,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
    self.optobj=optobj
    self.vp = self.optobj.vp
    self.vr = self.optobj.vr
    self.direction = 1
    self.freq = 2.0
    self.maxFreq = 3.0
    self.turn = 0
    self.roll = 0
    self.yaw = 0
    self.phi_s = 0
    self.t_s = 0
    self.maxTurn = 1.0
    self.last = time.time()
    self.c = polo.Cluster()
    self.c.init()

    #initial simplex (frequency,roll,yaw,phi_s, t_s, ecc)
    self.c.d_buehler_mode = True
    self.x0 = array([[3.5,0.4,0.9,pi/5,0.25,0],
                     [2.0,0.4,0.9,pi/5,0.25,0],
                     [3.5,0.2,0.9,pi/5,0.25,0],
                     [3.5,0.4,0.5,pi/5,0.25,0],
                     [3.5,0.4,0.9,pi/3,0.25,0],
                     [3.5,0.4,0.9,pi/5,0.12,0],
                     [3.5,0.4,0.9,pi/5,0.25,math.pi/2]])

    self.x_low_bound = array([0,0,0,0,0,-1.0])
    self.x_up_bound = array([float('Inf'), 1.0, 1.0, pi, 0.5, 1.0])
    """
    #initial simplex (frequency,roll,yaw)
    self.x0 = array([[1.0,0.25,0.25],
                     [2.0,0.25,0.25],
                     [1.0,0.75,0.25],
                     [1.0,0.25,0.75]])
    result simplex: 3.5, 0.4, 0.9
    #initial simplex (frequency)
    self.x0 = array([[1.5],[2.5]]) 
    """


  def fun( self, t ):
    now = time.time()

    # Rate limit the position commands
    dt = now - self.last
    if dt <0.005: #J seems I need to half this to get this to work??
      return
    #self.plan.setFrequency(self.direction*self.freq)
    self.c.turn = self.turn
    self.c.rollAmp = self.roll
    self.c.yawAmp = self.direction*self.yaw
    self.c.phi_s = self.phi_s
    self.c.t_s = self.t_s

    #progress("turn: %f, yawamp: %f"%(self.c.turn, self.c.yawAmp))
    self.c.setpos(t)
    self.last = time.time()

  def onStart(self):
    self.opt = fminIter(self.x0,xtol=0.2)
    self.val = self.opt.next() # Initializes val to an empty list           
    #J I'm leaving out loading an old optim for now
    self.x = self.opt.next()

    self.plan = FunctionCyclePlan(self, self.fun,180) 
    self.ra = RunAcrossPlan(self,self.optobj)

    self.plan.onStart = curry(progress,">>> START")
    self.plan.onStop = curry(progress,">>> STOP")
    self.plan.setFrequency(self.direction*self.freq)

    sf = StickFilter(self)
    sf.setLowpass("joy0axis2",10)
    sf.setLowpass("joy0axis3",10)
    sf.start()
    self.sf = sf
    self.timeToShow = self.onceEvery(0.25)
    
  def onStop(self):
    progress("turning off robot")
    self.c.zeros()
    self.c.off()
    self.optobj.vpw.close()

  def onEvent(self, evt):
    if self.timeToShow():
      pass

    if evt.type==JOYBUTTONDOWN and evt.joy==0:
      progress( describeEvt(evt) )

      if evt.button==5: #button 6 on logitech
        # start
        self.x = self.opt.next() #get next simplex to try
        #check limits:
        while(self.x<self.x_low_bound).any() or (self.x>self.x_up_bound).any():
          self.val[-1] = 100 # if outside limits give high cost
          self.x = self.opt.next()
          
        #update centipede cluster parameters with new simplex:
        self.plan.setFrequency(self.x[0])
        self.roll = self.x[1]
        self.yaw = self.x[2]
        self.phi_s = self.x[3]
        self.t_s = self.x[4]
        self.ecc = self.x[5]


        self.xy_start = complex(nanMean(self.optobj.x),nanMean(self.optobj.y))
        self.t_start = time.time()
        self.ra.start()
        progress('--> starting run across')

      if evt.button==7: #button 8 on logitech
        # stop
        progress('Emergency stop, give high cost (logitech 2) or just reset')
        self.ra.stop()
        progress('STOP')

      if evt.button == 0:  #button 1 on the logitech
        # accept for Nelder-Mead
        progress('accept for Nelder-Mead')
        dist = cmath.polar(self.xy_end-self.xy_start)[0]/1000
        self.val[-1] = (self.t_end - self.t_start)/dist #calc cost
        #output to file:
        #self.optobj.fd.write("sim:[ %.4f, %4f, %4f], cost:  %.4f\n"%(self.x[0],
        #                             self.x[1], self.x[2], self.val[-1]))
        self.optobj.fd.write("sim:[ %.4f, %.4f, %.4f, %.4f, %.4f, %.4f], cost:  %.4f\n"%(
            self.x[0], self.x[1],self.x[2],self.x[3],self.x[4],self.x[5],self.val[-1]))
        self.optobj.fd.flush()

      if evt.button == 1:
        #bad gait, give high cost.
        progress('bad gait, give high cost.')
        self.val[-1] = 100
        #output to file:
        self.optobj.fd.write("sim:[ %.4f, %.4f, %.4f, %.4f, %.4f, %.4f], cost:  %.4f\n"%(
            self.x[0], self.x[1],self.x[2],self.x[3],self.x[4],self.x[5],self.val[-1]))
        self.optobj.fd.flush()
    if evt.type==KEYDOWN:
      if evt.key==ord('d'): # 'd' reverses direction
        self.direction = -self.direction
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
  import joy
  joy.DEBUG[:]=[]


  optobj = OptimClass()
  logname = raw_input("Please enter desired optim log file name\n").lower()
  optobj.fd, optobj.fr = logOpen(logname, 'optimlog.txt') # Should this be within OptimClass?
  logname = raw_input("Please enter desired vicon log file name\n").lower()
  optobj.vpw, optobj.vpr = logOpen(logname, 'viconlog.txt') #store vicon data
  optobj.initialize()
  app=OptimizerApp(optobj)
  app.run()

