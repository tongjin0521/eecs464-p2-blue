from joy import *

class GaitCyclePlanApp( JoyApp ):
  SHEET = loadCSV("demos/M4ModLab.csv")
  
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,scr={},*arg,**kw)
    
  def onStart( self ):
    self.plan = GaitCyclePlan( self, 
      self.SHEET, x='>catX',y=('>catY',lambda v : -v))
    self.plan.setPeriod(10)
  
  def onEvent(self, evt):
    if evt.type==KEYDOWN:
      if evt.key in [ord('q'),27]: # 'q' and [esc] stop program
        self.stop()
        #
      elif evt.key==ord(' '): # [space] toggles motion
        if self.plan.isRunning():
          self.plan.stop()
          progress('Stopped motion')
        else:
          self.plan.start()
          progress('Started motion')
      
if __name__=="__main__":
  print """
  Demo of SheetPlan class
  -----------------------
  
  Use this demo with the demo-plans.sb Scratch project:
  $ scratch demos/demo-plans.sb &
  
  When any key is pressed, starts a SheetPlan making the 
  cat move around a shape defined in the CSV file.

  The application can be terminated with 'q' or [esc]
  """
  import joy
  app=GaitCyclePlanApp()
  app.run()

