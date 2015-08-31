from joy import *
from time import time as now
import ckbot.dynamixel as DX
import ckbot.logical as logical
import ckbot.nobus as NB
import math
import numpy
from qualisys import QTM, EndOfDataPacket
from ckbot.ckmodule import progress


import sys

from centipedeContact import gaitParams, contactGait

class QTMSink( Plan ):
  """
  Concrete class QTMSink
 
  """
  DEFAULT_NETWORK = ('192.168.254.1')
 
  def __init__( self, app, net=DEFAULT_NETWORK, rate=500 ):
        """
        Attributes:
          net -- tuple -- parameters for QTM.connectOnNet
          rate -- integer -- maximal number of events processed each time-slice
        """
        Plan.__init__(self, app)
        self.net = net
        self.qtm = QTM()
        self.rate = rate
        self.pt_num = 4 #int(sys.argv[2])
        self.last_centr = [0]*self.pt_num
        #self.initCentroid = asfarray([0,0,0])
        #self.result_n = open(self.fname,'a')
        #dname = "result.txt"
        #self.dist_n = open(dname, 'a')
        
        
 
  #Calculate distance between two centroid points
  def calc_Distance (self, new_centroid, old_centroid):
        distance = sqrt(((old_centroid[0] - new_centroid[0])**2) + ((old_centroid[1] - new_centroid[1])**2) + ((old_centroid[2] - new_centroid[2])**2))
            

        if distance >= TEST_DIST:
          progress("Hexapod completed run")
          #self.dist_n.write("%s" % (str(distance)))
          app.writeResult(distance)
          app.onStop()
          self.onStop()
        else: return False
        
 
  def check_nan_start(self, points, point_num):

        
        for i in range(0, point_num):
            nan = False
            try:
              int(points[i][0])
              
            except:
              nan = True
              break

        #things.write("%s %s" % (str(points[i][0]), '\n'))
        if nan:
            progress("Error: Points must be labeled in QTM on start of Qualisys")
            app.onStop()
            sys.exit(1)



  def check_nan_in_run(self, points, point_num):
    
    nan_thresh = point_num/2

    nan_count = 0

    for i in range(0, point_num):
      try:
        int(points[i][0])
      except:
        nan_count+=1

    if nan_count > nan_thresh:
      progress("Error: lost at least half of tracking points")
      progress("robot may be out of tracking area")

      app.onStop()
      sys.exit(1)
  

  #adds the position of all labled tracking dots to two arrays of differnt 
  #types for use and export
  def add_frame(self, points, point_num):


    self.data_flat = numpy.concatenate((self.data_flat, points), axis=0)
    print("did it*************************************************")
    self.count += 1

    


  def onStart( self ):
        progress("onStart")
        Q = self.qtm
        try: Q.connectOnNet(self.net)
        except:
          progress("SocketTimeout: QTM Connection Timed Out")
          print("SocketTimeout: QTM Connection Timed Out")
          app.onStop()
          sys.exit(1)
        Q.command("Version 1.9")
        Q.command("StreamFrames AllFrames UDP:%d 3D" % (Q.udp.getsockname()[1]))
        Q.udp.settimeout(0.0)


        cc = self.qtm.pudp.next()
        while cc is None:
          cc = self.qtm.pudp.next()
    
        if (type(cc) == EndOfDataPacket):
            progress("Error: Start Qualisys capture")
            app.OnStop()
            sys.exit(1)


        dd = cc.comp[0]
         
        self.count = 0
        self.data_flat = numpy.copy(dd.mrk)
        print(self.data_flat.shape)
        #self.data_flat.resize(1, self.pt_num * 3)
        #print self.data_flat
      
        
        self.check_nan_start(dd.mrk, self.pt_num)
        #self.data = self.check_nan(dd.mrk, self.pt_num, self.last_centr)
        
          
        
    
    
  def onStop( self ):
        progress("Closed QTM")
        self.qtm.close()
        print(self.count)
        out = numpy.reshape(self.data_flat, (-1, 5))
        print(out.shape)
        print(out)
        numpy.savetxt('testout.txt', out, delimiter=',')
        
 
  def onEvent( self, evt ):
        progress("onEvent")
        if evt.type != TIMEREVENT:
          return False


        for k in xrange(self.rate):
          cc = self.qtm.pudp.next()
          if cc is None:
            break
            
          dd = cc.comp[0]
          self.check_nan_in_run(dd.mrk, self.pt_num)
          self.add_frame(dd.mrk, self.pt_num)
 

        return False
 

class Segment(object):
  def __init__(self, front, back, leg):
    self.f = front
    self.b = back
    self.l = leg
        
  def set_pos(self,yaw,bend,roll):
    A = numpy.matrix([[2,0.5,0],[-2,0.5,0],[0,0,1]],dtype='float') 
    #numpy.matrix([[2,0.5,0],[-2,0.5,0],[0,0,1]],dtype='float')
    pos=A*numpy.matrix([[yaw],[bend],[roll]])
    if self.f!=None:
      self.f.set_pos(pos[0])
    if self.b!=None:
      self.b.set_pos(pos[1])
    if self.l!=None:
      self.l.set_pos(pos[2])

class MechapodApp( JoyApp ):

  def __init__(self,*arg,**kw):
    assert not kw.has_key('cfg')
    cfg = dict(
       maxFreq = 3,
       freq = 1
     )
    
    JoyApp.__init__(self, confPath="$/cfg/mechapod2.yml", cfg=cfg, *arg,**kw)
    self.stickMode = False
    self.last = now()
    self.backward = 0

  def setParams(self, gp, back):
    self.gp = gp
    self.backward = back
    

  def _tripodGaitFun( self, phi ):  		      
    g1 = self.gait1
    g2 = self.gait2
    g3 = self.gait3
    
    #for legs 1 & 3
    g1.manageGait(phi)
    s1roll = math.degrees(g1.roll)
    s1yaw = math.degrees(g1.yaw)

    
    g3.manageGait(phi)
    s3roll = math.degrees(g3.roll)
    s3yaw = math.degrees(g3.yaw)

    if self.backward == 1:
      s1yaw = -s1yaw
      s3yaw = -s3yaw
	
    #for leg 2: give phi2 = (phi1 + 0.5) mod 1
    if(phi > 0.5):
        phi2 = phi -0.5
    else:
        phi2 = phi + 0.5
        
    g2.manageGait(phi2) 
    s2roll = math.degrees(g2.roll)
    s2yaw = math.degrees(g2.yaw)
    
    #s2roll = -s2roll
    #s2yaw = -s2yaw
    


    #roll/yaw are deg, need to convert to centidegrees
    self.S1.set_pos(s1yaw*100, g1.bend, s1roll*100)  
    self.S2.set_pos(s2yaw*100, g2.bend, s2roll*100)   
    self.S3.set_pos(-s3yaw*100, -g3.bend, -s3roll*100)      #facing opposite     


  def onStart(self):
    self.S1 = Segment(None, self.robot.at.B1, self.robot.at.L1)
    self.S2 = Segment(self.robot.at.F2, self.robot.at.B2, self.robot.at.L2)
    self.S3 = Segment(self.robot.at.F3, None, self.robot.at.L3)
    #set the slope parameter for each motor:
    for m in self.robot.itermodules():
      if isinstance( m, DX.MX64Module ):
        continue
      m.mem[m.mcu.ccw_compliance_slope] = 64
      m.mem[m.mcu.cw_compliance_slope] = 64

    self.last = 0
    self.stickMode = False
    
    #setup parameters for contact gait - reasonable vals for hexapod demo

    
    self.gait1 = contactGait(self.gp, self.S1.l)  #+
    self.gait2 = contactGait(self.gp, self.S2.l)
    self.gait3 = contactGait(self.gp, self.S3.l)

    self.tripodGait = FunctionCyclePlan(self, self._tripodGaitFun, N=302,interval=0.02)
    self.tripodGait.setFrequency(self.cfg.freq)
    
    sf = StickFilter(self)
    sf.setLowpass("joy0axis0",10)
    sf.setLowpass("joy0axis1",10)
    sf.start()
    self.sf = sf
    
    self.timeToShow = self.onceEvery(0.25)
    self.timeToUpdate = self.onceEvery(0.05)

    self.qtms = QTMSink(self)

    self.qtms.start()

  def onStop(self):
    self.qtms.onStop()
    self.robot.off()
  
  def onEvent(self, evt):
    tg = self.tripodGait
    # If it's time, display status message
    if self.timeToShow():
      progress('freq: %g, bend: %g, stickMode: %g' 
               % (tg.getFrequency(), self.gait1.bend, self.stickMode))
    # If it's time, process stick mode commands and send into robot
    if self.timeToUpdate():
      if self.stickMode:
        #update gait with joystick inputs
        bend = self.sf.getValue("joy0axis0") #value range: [-1, 1]
        freq = self.sf.getValue("joy0axis1") #value range: [-1, 1]
        tg.setFrequency(freq * self.cfg.maxFreq)
        b = self.cfg.maxBend * bend
        self.gait1.bend = b #+
	self.gait2.bend = b
        self.gait3.bend = b
      else:
        #return to button control
        tg.setFrequency(self.cfg.freq) #self.plan.setFrequency(self.cfg.freq) 
        self.gait1.bend = 0
        self.gait2.bend = 0
        self.gait3.bend = 0
    # Ignore any other timer events
    if evt.type==TIMEREVENT:
	return
    if evt.type == JOYAXISMOTION:
      # Make sure StickFilter gets all JOYAXISMOTION events
      self.sf.push(evt)
      return
    # 
    #  Process other user control commands 
    #
    if ((evt.type==KEYDOWN and evt.key==K_m)
	or (evt.type==JOYBUTTONDOWN and evt.button==5)): # r2 button
          tg.start()#
	  progress("(say) Starting tripod gait")
    elif ((evt.type==KEYDOWN and evt.key==K_s)
	or (evt.type==JOYBUTTONDOWN and evt.button==7)): # r1 button
          tg.stop()
	  progress("(say) Stopping tripod gait")
	  tg.setFrequency(0)
    elif (evt.type==JOYBUTTONDOWN and evt.button==0):
        self.stickMode = not self.stickMode   
    # Punt to superclass
    return JoyApp.onEvent(self,evt)


if __name__=="__main__":
  print """
  Centipede
  -------------------------------
   
  When any key is pressed, a 7-module centipede commences locomotion.

  The application can be terminated with 'q' or [esc]
  """
  # for actual operation use w/ arch=DX & NO required line:  
  ckbot.logical.DEFAULT_BUS = ckbot.dynamixel
  ckbot.logical.DEFAULT_PORT = dict(TYPE="tty", baudrate=115200, glob="/dev/ttyUSB*")
  
  # for simulation use w/ arch=NB & required line:
  #ckbot.logical.DEFAULT_BUS = ckbot.nobus

  #modify gait params before initialization
  gp = gaitParams()
  gp.rollThresh = -(24*pi/180) #24
  gp.yawThresh = -(7*pi/180)  #7
  gp.maxRoll = (26*pi/180) #26
  gp.rollAmp = (26*pi/180)
  gp.yawAmp = (8*pi/180) #8
  gp.stanceVel = 1.26

  back = 1  #runs gait backward if 1 forward if 0
  
  app=MechapodApp(robot=dict(
    arch=DX,count=7,fillMissing=True,
    #required=[ 0x26, 0x08, 0x06, 0x2D, 0x31, 0x27, 0x22 ] 
    ))

  app.setParams(gp, back)
  
  app.run()

