from joy import Plan, JoyApp, FunctionCyclePlan, curry, Plan, StickFilter, JOYBUTTONDOWN, KEYDOWN, JOYAXISMOTION, TIMEREVENT, describeEvt
from numpy import asfarray,dot,floor,abs,sqrt, matrix
import time as now
from os.path import exists
from re import findall
from centipedeContact import contactGait, gaitParams
#import centipedeTurnInPlace as centTIP
import math
from qualisys import QTM, EndOfDataPacket
from ckbot.ckmodule import progress


import sys

TEST_DIST = 800
file_name = "result.csv"


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
        self.pt_num = int(sys.argv[2])
        #self.fname = "r"
        self.last_centr = [0]*self.pt_num
        self.initCentroid = asfarray([0,0,0])
        #self.result_n = open(self.fname,'a')
        #dname = "result.txt"
        #self.dist_n = open(dname, 'a')
        
        

  #Calculate the centroid of the points (points, meaning markers)
  def calc_Centroid( self, points, point_num): #pass in dd.mrk, number of points
        x_tot = 0
        y_tot = 0
        z_tot = 0
         
        for i in range(0, point_num):
          x_tot += float(points[i][0])
          y_tot += float(points[i][1])
          z_tot += float(points[i][2])
          
        x = x_tot / point_num
        y = y_tot / point_num
        z = z_tot / point_num
        centroid = asfarray([x, y, z])
        return centroid
 
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
 
  def check_nan(self, points, point_num, last):
        new = []
        for i in range(0, point_num):
          nan = False
          try:
            int(points[i][0])
          except:
             new.append(last[i])
             nan = True
          if not nan:
            new.append(points[i])         
        return new
 
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
   
        self.check_nan_start(dd.mrk, self.pt_num)
        centr = self.check_nan(dd.mrk, self.pt_num, self.last_centr)
    
        self.init_Centroid = self.calc_Centroid(centr, self.pt_num)
    
        self.last_centr = self.init_Centroid
    
  def onStop( self ):
        progress("Closed QTM")
        self.qtm.close()
        app.onStop()
        sys.exit()
 
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
          self.last_centr = self.check_nan(dd.mrk, self.pt_num, self.last_centr)
 
          new_centroid = self.calc_Centroid(self.last_centr, self.pt_num)
          progress("-------------")
          progress(new_centroid)
          self.calc_Distance(self.init_Centroid, new_centroid)


        return False
 
"""
Running the hexapod


"""


#class Struct(object):
 # def __init__(self,**kw):
  #      self.__dict__.update(kw)


class Segment(object):
  def __init__(self, front, back, leg, fs=1, bs=1, ls=1):
        """Set the module IDs and directions of the modules in a segment"""
        self.f = front
        self.b = back
        self.l = leg
        self.fs = fs
        self.bs = bs
        self.ls = ls


  def set_pos(self,yaw,bend,roll, b):
    A = matrix([[2,0.5,1],[-2,0.5,1],[0,0,1]],dtype='float') 
    #numpy.matrix([[2,0.5,0],[-2,0.5,0],[0,0,1]],dtype='float')
    pos=A * matrix([[yaw],[bend],[roll]])
    if self.f!=None:
      self.f.set_pos(pos[0])
    if self.b!=None:
      self.b.set_pos(pos[1])
    if self.l!=None:
      self.l.set_pos(b * pos[2])

  def set_pos_2(self,yaw,bend,roll, b):
    A = matrix([[2,0.5,1], [-2,0.5,1],[0,0,1]],dtype='float')
    pos=A * matrix([[yaw],[bend],[roll]])
    if self.f!=None:
      self.f.set_pos(-pos[0])
    if self.b!=None:
      self.b.set_pos(pos[1])
    if self.l!=None:
      self.l.set_pos(b* pos[2])

class MechapodApp( JoyApp ):

  def __init__(self,*arg,**kw):

    assert not kw.has_key('cfg')
    cfg = dict(
       maxFreq = 3,
       freq = 1
     )
    JoyApp.__init__(self, confPath="$/cfg/mechapod2.yml", cfg=cfg, *arg,**kw)
    self.stickMode = False
    self.last = now.time()
    self.backward = 0
    self.fout = open(file_name, 'a')

  #import parameters for the gait for each run gp is a gaitParams() object  
  def setParams(self, gp, back):
    self.gp = gp
    #self.gp.maxFreq= .3
    #self.gp.maxBend = 900
    #self.gp.freq = 1
    if back == 'back':
      self.backward = 1
      
  def _tripodGaitFun( self, phi ):  		      
    
    if self.backward == 1:
      phiB = -phi
      b = -1
    else:
      phiB = phi
      b = 1

    g1 = self.gait1
    g2 = self.gait2
    g3 = self.gait3
    
    #for legs 1 & 3
    g1.manageGait(phiB)
    s1roll = math.degrees(g1.roll)
    s1yaw = math.degrees(g1.yaw)
    
    g3.manageGait(phiB)
    s3roll = math.degrees(g3.roll)
    s3yaw = math.degrees(g3.yaw)

		
    #for leg 2: give phi2 = (phi1 + 0.5) mod 1
    if(phi > 0.5):
        phi2 = phi -0.5
    else:
        phi2 = phi + 0.5
        
    if self.backward == 1:
      phi2B = -phi2
    else:
      phi2B = phi2

    g2.manageGait(phi2B) 
    s2roll = math.degrees(g2.roll)
    s2yaw = math.degrees(g2.yaw)
    

    #roll/yaw are deg, need to convert to centidegrees
    self.S1.set_pos(s1yaw*100, g1.bend, s1roll*100, b)  
    self.S2.set_pos_2(s2yaw*100, g2.bend, s2roll*100, b)   
    self.S3.set_pos(-s3yaw*100, -g3.bend, -s3roll*100, b)      #facing opposite     


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
    
    #removed gait params, they are set from outside
    #SEE self.setParams()
    
    self.gait1 = contactGait(self.gp, self.S1.l)  #+
    self.gait2 = contactGait(self.gp, self.S2.l)
    self.gait3 = contactGait(self.gp, self.S3.l)

 #this is where the mesurements and changes were


    self.tripodGait = FunctionCyclePlan(self, self._tripodGaitFun, N=302,interval=0.02)
    self.tripodGait.backward = self.backward
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
    #straightens the robot before each run
    for m in [self.robot.at.B1,self.robot.at.F2,self.robot.at.B2,self.robot.at.F3]:
      m.set_pos(0)
      for k in xrange(63):
        s = math.sin(k/10.0)
        r = int(math.sqrt(abs(s))*math.copysign(1,s)*4000)
        #print r
        self.robot.at.L1.set_pos(-r)
        self.robot.at.L2.set_pos(r)
        self.robot.at.L3.set_pos(r)
        #now.sleep(0.015)

    self.robot.at.B1.set_pos(0)
    self.robot.at.L1.set_pos(0)
    self.robot.at.F2.set_pos(0)
    self.robot.at.B2.set_pos(0)
    self.robot.at.L2.set_pos(0)
    self.robot.at.F3.set_pos(0)
    self.robot.at.L3.set_pos(0)
    
    self.robot.off()

    now.sleep(1)

    self.startTime = now.clock()
    self.tripodGait.start()
    
  def onStop(self):
        progress("Stopped robot")

        self.S1.set_pos(0, 0, 0, 1)  
        self.S2.set_pos(0, 0, 0, 1)   
        self.S3.set_pos(0, 0, 0, 1)
        self.robot.off()
        #sys.exit()
           
  def onEvent(self, evt):
    pass


  def writeResult(self, distance):
    endTime = now.clock()
    totalTime = endTime - self.startTime
    self.fout.write("%f, %f, %d, %f, %f, %f %s" % (distance, totalTime, self.backward, self.gp.rollAmp, self.gp.yawAmp, self.gp.stanceVel, '\n'))
    
if __name__=="__main__":
  import ckbot.dynamixel as DX
  import ckbot.logical as L
  print """
  Centipede
  -------------------------------
   
  When any key is pressed, a 7-module centipede commences locomotion.


  The application can be terminated with 'q' or [esc]
  """

  
 
  robot = None
  
  L.DEFAULT_BUS = DX
  L.DEFAULT_PORT = dict(TYPE="tty", baudrate=1000000, glob="/dev/ttyUSB*")
 

  #calculate params based on agruments fed per run
  gp = gaitParams()
  gp.maxRoll = (26*math.pi/180) #26
  gp.rollAmp = float(sys.argv[4])*math.pi/180
  gp.yawAmp = float(sys.argv[3])*math.pi/180 #8
  gp.stanceVel = float(sys.argv[5])*math.pi/180
  gp.rollThresh = gp.rollAmp * -24 / 25.0 # default
  gp.yawThresh = gp.yawAmp * -3 / 5.0 # defa

  app=MechapodApp(robot=dict(
      arch=DX,count=7
    ))

  app.setParams(gp, sys.argv[6])
  
  app.run()
