from joy import Plan, JoyApp, FunctionCyclePlan, curry, Plan, StickFilter, JOYBUTTONDOWN, KEYDOWN, JOYAXISMOTION, TIMEREVENT, describeEvt
from numpy import asfarray,dot,floor,abs,sqrt
from time import sleep, time as now
from os.path import exists
from re import findall
from centipedeContact import ContactGait, GaitParams
import centipedeTurnInPlace as centTIP
from math import pi, degrees
from qualisys import QTM, EndOfDataPacket
from ckbot.ckmodule import progress


import sys


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
        self.fname = "result"+sys.argv[1]
        self.last_centr = [0]*self.pt_num
        self.initCentroid = asfarray([0,0,0])
        self.result_n = open(self.fname,'w')
        dname = "dist" + sys.argv[1]
        self.dist_n = open(dname, 'w')


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
        progress("%s %d" % ("Dist: ", distance))
        self.result_n.write("%s %s" % (str(now()),  '\n'))
        progress("Time written")
        if distance >= 1500:
          progress("Hexapod completed run")
          self.dist_n.write("%s %s" % (str(distance),  '\n'))
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
        if nan:
            progress("Error: Points must be labeled in QTM on start of Qualisys")
            sys.exit()


  def onStart( self ):
        progress("onStart")
        Q = self.qtm
        try: Q.connectOnNet(self.net)
        except:
          progress("SocketTimeout: QTM Connection Timed Out")
          print("SocketTimeout: QTM Connection Timed Out")
          app.onStop()
        Q.command("Version 1.9")
        Q.command("StreamFrames AllFrames UDP:%d 3D" % (Q.udp.getsockname()[1]))
        Q.udp.settimeout(0.0)


        cc = self.qtm.pudp.next()
        while cc is None:
          cc = self.qtm.pudp.next()
    
        if (type(cc) == EndOfDataPacket):
            progress("Error: Start Qualisys capture")
            sys.exit()


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
          self.last_centr = self.check_nan(dd.mrk, self.pt_num, self.last_centr)
 
          new_centroid = self.calc_Centroid(self.last_centr, self.pt_num)
          progress("-------------")
          progress(new_centroid)
          self.calc_Distance(self.init_Centroid, new_centroid)


        return False
 
"""
Running the hexapod


"""


class Struct(object):
  def __init__(self,**kw):
        self.__dict__.update(kw)


class Segment(object):
  def __init__(self, front, back, leg, fs=1, bs=1, ls=1):
        """Set the module IDs and directions of the modules in a segment"""
        self.f = front
        self.b = back
        self.l = leg
        self.fs = fs
        self.bs = bs
        self.ls = ls


  def set_pos(self,yaw,bend,roll):
        ybr = asfarray([yaw,bend,roll])
        if self.f!=None:
          self.f.set_pos(self.fs * dot([2,0.5,0],ybr))
        if self.b!=None:
          self.b.set_pos(self.bs * dot([-2,0.5,0],ybr))
        if self.l!=None:
          self.l.set_pos(self.ls * ybr[2])


class FunctionCyclePlanApp( JoyApp ):


  def __init__(self, *arg,**kw):


        JoyApp.__init__(self, confPath="$/cfg/JoyAppCentipedeV2.yml", *arg,**kw)
        self.turnInPlaceMode = 0
        self.backward = 0
        self.gaitSpec = None
        self.stickMode = False
    
        self.last = now()
        self.yaw_zero = False


  def fun( self, phase):                  
             
        g1 = self.gait1
        g2 = self.gait2
        g3 = self.gait3
    
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


        if(self.backward == 1):
          s1roll = -s1roll
          s2roll = -s2roll
          s3roll = -s3roll           
        else:
          pass


        # NOTE: angles need conversion from degree to centidegrees


        self.S1.set_pos(s1yaw*100, g1.bend, s1roll*100)  
        self.S2.set_pos(s2yaw*100, g2.bend, s2roll*100)   
        self.S3.set_pos(s3yaw*100, g3.bend, s3roll*100)  


 
  def onStart(self):
        if len(self.robot)<7:
            raise RuntimeError('Could not find all modules')
        self.S1 = Segment(None, self.robot.at.B1, self.robot.at.L1)
        self.S2 = Segment(self.robot.at.F2, self.robot.at.B2, self.robot.at.L2)
        self.S3 = Segment(self.robot.at.F3, None, self.robot.at.L3, ls=-1) # our current robot has an axis flipped
            


        self.last = 0
        self.backward = 0
        self.stickMode = False


        #setup parameters for contact gait - reasonable vals for hexapod demo
        initParamsDemo = GaitParams()
        initParamsDemo.rollThresh = -(25*pi/180)
        initParamsDemo.yawThresh = -(8*pi/180)
        initParamsDemo.rollAmp = (26*pi/180)
        initParamsDemo.yawAmp = (9*pi/180)
        initParamsDemo.stanceVel = 1.26
    
        self.gait1 = ContactGait(initParamsDemo, self.S1.l)
        self.gait2 = ContactGait(initParamsDemo, self.S2.l)
        self.gait3 = ContactGait(initParamsDemo, self.S3.l)
            


        #Update the struct with the values in param_n
        trialnum = sys.argv[1]
        fname = "param"+str(trialnum)
        if exists(fname):
          fparam = open(fname,'r')
          all_params = []
          for line in fparam:
            found = findall('[-+]?\d*\.\d+|[-+]?\d+', line)
            all_params.append(float(found[0]))  
 
          s = Struct(
            freq = 1, # default; in units of maxFreq
            rollAmp = (30*pi/180),
            yawAmp = (13*pi/180),
            stanceVel = 3, #3                
            maxFreq = .3, # Hz
            maxBend = 900 # centiDegree
          )
          print all_params
          s.rollAmp, s.yawAmp, s.freq, s.stanceVel = all_params
          #s.rollAmp, s.yawAmp, s.stanceVel = all_params[:-1]
          s.rollThresh = s.rollAmp * -23 / 30.0 # default
          s.yawThresh = s.yawAmp * -12 / 13.0 # default
          #s.rollAmp, s.yawAmp, s.yawThresh, s.rollThresh, s.stanceVel = all_params[:-1]
          # s.freq = all_params[-1]
          self.gaitSpec = s
        else:
            progress("Error: Param file does not exist")   
            sys.exit()


        self.plan = FunctionCyclePlan(self, self.fun,N=302,interval=0.02)
        self.plan.onStart = curry(progress,">>> START")
        self.plan.onStop = curry(progress,">>> STOP")
        self.plan.setFrequency(self.gaitSpec.freq)
        self.dir = 1


        self.timeToShow = self.onceEvery(0.25)
        self.timeToPlot = self.onceEvery(0.5)
        self.qtms = QTMSink(self)
    
        self.qtms.start()
        self.plan.start()
    
    
  def onStop(self):
        progress("Stopped robot")
        self.S1.set_pos(0, 0, 0)  
        self.S2.set_pos(0, 0, 0)   
        self.S3.set_pos(0, 0, 0)
        self.robot.off()
        sys.exit()
           
  def onEvent(self, evt):
        gs = self.gaitSpec
    
        if evt.type==JOYBUTTONDOWN and evt.joy==0:    
            if evt.button == 1: # Circle
                self.robot.off()
                self.plan.stop()
                #progress('STOP')
                gs.freq = 0
            
        self.stickMode = False
        bend = 0  
         
        freq = 0.5 if (sys.argv[3] == 'back') else -0.5
        maxFreq = gs.maxFreq
        maxBend = gs.maxBend
          
        #handle negative frequency
        if(freq < 0):
            freq = -1*freq
            self.backward = 1
        else:
            self.backward = 0
            self.gaitSpec.freq = maxFreq*freq #for plotting purposes
            self.plan.setFrequency(self.gaitSpec.freq)
            
            self.gait1.bend = (maxBend*bend)
            self.gait2.bend = (maxBend*bend)
            self.gait3.bend = (maxBend*bend)
 
        if evt.type!=TIMEREVENT:
          JoyApp.onEvent(self,evt)
 
    
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
  L.DEFAULT_PORT = dict(TYPE="tty", baudrate=57600, glob="/dev/ttyUSB*")
 
  app=FunctionCyclePlanApp(robot=dict(
      arch=DX,count=7
    ))
  app.run()