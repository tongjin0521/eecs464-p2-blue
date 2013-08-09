from joy import *

class SheetPlanApp( JoyApp ):
  SHEET = loadCSV("demos/M4ModLab.csv")
  
  def __init__(self,bind,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
    self.bind = bind
    
  def onStart( self ):
    self.plan = SheetPlan( self, self.SHEET, **self.bind)
  
  def onEvent(self, evt):
    if evt.type==KEYDOWN and evt.key in [27,ord('q')]:
      self.stop()
    elif evt.type==KEYDOWN or evt.type==JOYBUTTONDOWN:
      self.plan.start()

if __name__=="__main__":
  print """
  Demo of SheetPlan class
  -----------------------
  
  Can be used in three modes:
  
  In default, debug mode -- outputs to the screen
  
  With a robot, use the --withRobot or -r switches and set up your JoyApp.yml to map some robot nodes to 'front' and 'rear'. Those nodes will move according to the gait table in the csv file.
  
  With scratch,  use the --withScratch or -s switches, after starting Scratch with the demo-plans.sb Scratch project in another window. Click on the green flag to start the cat drawing. The cat will move according to the gait table in the csv file.
  
  When any key is pressed, starts the SheetPlan moving through the gait table.

  The application can be terminated with 'q' or [esc]
  """
  import sys
  app = None
  if len(sys.argv)>1:
    if sys.argv[1]=='--withRobot' or sys.argv[1]=='-r':
      app = SheetPlanApp(bind=dict(
          x='front/@set_pos', y='rear/@set_pos'), 
        robot={} )
    elif sys.argv[1]=='--withScratch' or sys.argv[1]=='-s':
      app = SheetPlanApp(bind=dict(
          x='>catX', y='>catY'),
        scr={} )
  if app is None:
    app = SheetPlanApp(bind=dict(x="#x ", y="#y "))    
  app.run()

