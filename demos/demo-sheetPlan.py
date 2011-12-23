from joy import *

class SheetPlanApp( JoyApp ):
  SHEET = loadCSV("demos/gait.csv")
  
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
    
  def onStart( self ):
    if self.robot is not None:
      self.plan = SheetPlan( self, self.SHEET, 
        x='front/@set_pos', y='rear/@set_pos')
    elif self.scr is not None:    
      self.plan = SheetPlan( self, self.SHEET, 
        x='>catX', y='>catY')
    else:
      raise RuntimeError("Must initialize either Scratch or Robot outputs")
  
  def onEvent(self, evt):
    if evt.type==KEYDOWN and evt.key in [27,ord('q')]:
      self.stop()
    elif evt.type==KEYDOWN or evt.type==JOYBUTTONDOWN:
      self.plan.start()

if __name__=="__main__":
  print """
  Demo of SheetPlan class
  -----------------------
  
  Use this demo with the demo-plans.sb Scratch project, 
  or set up your JoyApp.yml to map robot nodes to 'front'
  and 'rear', and run the program with the commandline
  parameter '--withRobot' (or '-r'), e.g.
  
  python demo-sheetPlan.py --withRobot
  
  When any key is pressed, starts a SheetPlan making the 
  cat move around a square.

  The application can be terminated with 'q' or [esc]
  """
  import sys
  app = None
  if len(sys.argv)==3:
    if sys.argv[2]=='--withRobot' or sys.argv[2]=='-r':
      app = SheetPlanApp(robot={})
  if app is None:
    app = SheetPlanApp(scr={})    
  app.run()

