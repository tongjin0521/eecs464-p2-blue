from joy import *

class GaitCyclePlanApp( JoyApp ):
  SHEET = loadCSV("demos/M4ModLab.csv")
  
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,scr={},*arg,**kw)
    
  def onStart( self ):
    if self.robot is not None:
      self.plan = GaitCyclePlan( self, self.SHEET, maxFreq = 0.3,
        x='front/@set_pos', y='rear/@set_pos')
    elif self.scr is not None:    
      self.plan = GaitCyclePlan( self, self.SHEET, maxFreq = 0.3,
        x=('>catX',lambda x : x/2), y=('>catY',lambda x : -x/2))
    else:
      raise RuntimeError("Must initialize either Scratch or Robot outputs")
    self.plan.onStart = curry(progress,">>> START")
    self.plan.onStop = curry(progress,">>> STOP")
    self.plan.setFrequency(0.2)
  
  def onEvent(self, evt):
    if evt.type==KEYDOWN:
      if evt.key in [ord('q'),27]: # 'q' and [esc] stop program
        self.stop()
        #
      elif evt.key==ord(' '): # [space] stops cycles
        self.plan.setPeriod(0)
        progress('Stopped motion')
        #
      elif evt.key in (ord(','),ord('.')):
        f = self.plan.getFrequency()
        # Change frequency up/down in range -limit..limit hz
        if evt.key==ord(','):
          f -= 0.03
        else:
          f += 0.03
        self.plan.setFrequency(f)
        progress('Frequency changed to %.2f Hz' % self.plan.getFrequency())
        #
      elif evt.key==ord('h'):
        progress( "HELP: ',' to decrease/reverse period; '.' opposite; ' ' stop; 'q' end program; 'h' this help; other keys start Plan" )
        #
      elif evt.key in range(ord('0'),ord('9')+1):
        self.plan.setFrequency(0)
        phi = ('1234567890'.find(chr(evt.key)))/9.0
        self.plan.moveToPhase( phi )
        progress('Moved to phase %.2f' % phi)
      else: # any other key
        progress( "Starting cycles...." )
        self.plan.start()
        
    if evt.type!=TIMEREVENT:
      JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  print """
  Demo of SheetPlan class
  -----------------------
  
  Use this demo with the demo-plans.sb Scratch project.
  
  When any key is pressed, starts a SheetPlan making the 
  cat move around a square.

  The application can be terminated with 'q' or [esc]
  """
  import joy
  app=GaitCyclePlanApp()
  app.run()

