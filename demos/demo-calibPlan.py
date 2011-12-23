from joy import *

class CalibrationPlan( Plan ):
  def __init__(self,app,mod):
    Plan.__init__(self,app)
    self.mod = mod
    self.clicked = False
  
  def onEvent( self, evt ):
    if evt.type == KEYDOWN:
        self.clicked = True
    # Always run
    return True
  
  def behavior( self ):
	#send go limp command
    self.mod.go_slack()
	#command user to move module to limits
    mx = -1e9
    mn = 1e9
	#wait for user input (enter stroke or something)
    self.clicked = False
    while not self.clicked:
	  #remember min and max vals
      yield
      adc = self.mod.od.get_rawADC()
      if mx<adc:
        mx = adc
      if mn>adc:
	    mn = adc
      print "Move module to both limits then hit a key !!! min adc = " , mn , " max adc = " , mx
    #enter command mode
    self.mod.od.set_calmode(1)
    #send 0
    midVal = 0
    self.mod.od.set_pos(midVal)
    time.sleep(.5)
    #check if feedback is > max or < min and adjust
    while (self.mod.od.get_rawADC() <= mn+10) or (self.mod.od.get_rawADC() >= mx-10):
      #keep checking until successful
      if self.mod.od.get_rawADC() <= mn:
        midVal += 100
      else:
        midVal -= 100
      self.mod.od.set_pos(midVal)
      time.sleep(.1)
      print "!!! Adjusting middle value to" ,midVal
    #slowly increment until maximum pot value is reached
    mxRaw = midVal
    while self.mod.od.get_rawADC() < (mx):
	  #remember maximum raw value
      mxRaw += 25
      self.mod.od.set_pos(mxRaw)
      print "!!! Moving to " , mxRaw
      time.sleep(.025)
    #send 0
    self.mod.od.set_pos(midVal)
    #slowly decrement until minimum pot value is reached
    mnRaw = midVal
    while self.mod.od.get_rawADC() > (mn):
      #remember minimum raw value
      mnRaw -= 25
      self.mod.od.set_pos(mnRaw)
      print "!!! Moving to " , mnRaw
      time.sleep(.025)
    print "!!! Position min = ", mnRaw
    print "!!! Position max = ", mxRaw
    print "!!! Feedback min = ", mn
    print "!!! Feedback max = ", mx
    #raw amplitude = maximum raw - minimum raw
    print "!!! Pamp = ", (mxRaw-mnRaw)/2
    self.mod.od.set_calPamp((mxRaw-mnRaw)/2)
    #raw average = (maximum raw + minimum raw)/2
    print "!!! Pctr = ", (mxRaw+mnRaw)/2.0
    self.mod.od.set_calPctr((mxRaw+mnRaw)/2.0)
    #feedback amplitude = maximum feedback - minimum feedback
    print "!!! Famp = ", (mx-mn)/2
    self.mod.od.set_calFamp((mx-mn)/2)
    #feedback average = (maximum feedback + minimum feedback)/2
    print "!!! Fctr = ", (mx+mn)/2.0
    self.mod.od.set_calFctr((mx+mn)/2.0)
    #turn off raw mode
    self.mod.od.set_calmode(0)
    #send module reset command 
    
    #alert user it's done
    
class CalibrationApp( JoyApp ):
  
  def __init__(self,targetID,*arg,**kw):
    print targetID
    JoyApp.__init__(self,robot=dict(
      required=[targetID], # must find the target module
      names={ targetID : 'theModule' },
      walk=True # and walk its object dictionary
      ),*arg,**kw)
    
  def onStart( self ):
    self.plan = CalibrationPlan( self, self.robot.at.theModule )
    self.plan.start()
  
  def onEvent(self, evt):
    if evt.type==KEYDOWN:
      JoyApp.onEvent(self,evt)
      self.plan.push(evt)
      return
    if not self.plan.isRunning():
      self.stop()
      
if __name__=="__main__":
  print """
  Calibration tool
  ----------------

  Given a module ID (in hex) on the commandline, 
  calibrate this module.
  
  """
  import sys
  if len(sys.argv) != 2:
    sys.stderr.write("""
      <<insert usage message here>>>
      """)
    sys.exit(1)
  app=CalibrationApp(int(sys.argv[1],16))
  app.run()

