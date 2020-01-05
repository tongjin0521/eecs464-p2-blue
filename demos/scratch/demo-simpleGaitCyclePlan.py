'''
FILE demo-simpleGaitCyclePlan.py

THis file starts and stops a GaitCyclePlan on users input.
This file will use default CSV file to give outputs on scratch program
'''
from joy.decl import *
from joy import JoyApp, progress
from joy.plans import GaitCyclePlan
from joy.misc import loadCSV

class GaitCyclePlanApp( JoyApp ):
  '''
  This class starts a GaitCyclePlan with callbacks defined in a CSV file and gives
  output on the scratch window on users input

  It does that by starting a GaitCyclePlan with a binded scratch object and starts
  the plan on users input defined in onEvent

  This is useful if you want to execute a cycle using the data defined in a spreadsheet
  '''
  SHEET = loadCSV("M4ModLab.csv")

  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,scr={},*arg,**kw)

  #It creates a GaitCyclePlan based on callbacks defined in the SHEEt object
  def onStart( self ):
    #It parses the Sheet on initialization and executes the actions defined in _action function
    #which  _action function gives output to the scratch window
    self.plan = GaitCyclePlan( self,
      self.SHEET, x='>catX',y='>catY')
    #sets period to units
    self.plan.setPeriod(10)

  def onEvent(self, evt):
    if evt.type==KEYDOWN:
      if evt.key==K_SPACE: # [space] toggles motion
        if self.plan.isRunning():
          self.plan.stop()
          progress('Stopped motion')
        else:
          self.plan.start()
          progress('Started motion')
        return
    return JoyApp.onEvent(self,evt)

if __name__=="__main__":
  print ("""
  Demo of simple GaitCyclePlan class
  -----------------------

  Use this demo with the demo-plans.sb Scratch project:
  $ scratch demos/demo-plans.sb &

  When any key is pressed, starts a SheetPlan making the
  cat move around a shape defined in the CSV file.

  The application can be terminated with 'q' or [esc]
  """)
  import joy
  app=GaitCyclePlanApp()
  app.run()
