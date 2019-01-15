'''
FILE demp-sheetPlan.py

This file is used to start a plan using a given csv file and gives output in a mode defined 
by the user. It will parse the loaded csv file and bind correct output mode to give
appropriate output
'''
from joy.decl import *
from joy import JoyApp
from joy.plans import SheetPlan
from joy.misc import loadCSV


class SheetPlanApp( JoyApp ):
  '''
  This class starts a plan which takes input through a csv file

  It works by starting a SheetPlan object. After starting, __parseSheet function is called in SheetPlan object which
  parses the sheet and checks for possibles errors. After this, the bindings in self.bind are passed which are processed
  in the __initBindings method in plan class and appropriate behaviour is given as output

  It's useful for those who want to move along a predefined trajectory, or give a predefined motion. It expexts the motors
  to have mode=0 and names 'front' and 'rear' in the JoyApp.yml file
  '''
  SHEET = loadCSV('/home/birds/pyckbot/demos/M4ModLab.csv')
  #initializes the object
  def __init__(self,bind,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
    self.bind = bind
  #creates a plan  
  def onStart( self ):
    self.plan = SheetPlan( self, self.SHEET, **self.bind)
  
  def onEvent(self, evt):

    if evt.type==KEYDOWN and evt.key in [27,ord('q')]:
      self.stop()
    #starts the plan when any key is pressed
    elif evt.type==KEYDOWN or evt.type==JOYBUTTONDOWN:
      self.plan.start()

if __name__=="__main__":
  print """
  Demo of SheetPlan class
  -----------------------
  
  Can be used in three modes:
  
  In default, debug mode -- outputs to the screen
  
  With a robot, use the --withRobot or -r switches and set up your JoyApp.yml to map some robot nodes to 'front' and 'rear'. 
  Those nodes will move according to the gait table in the csv file.
  
  With scratch,  use the --withScratch or -s switches, after starting Scratch with the demo-plans.sb Scratch project in another window. 
  Click on the green flag to start the cat drawing. The cat will move according to the gait table in the csv file.
  
  When any key is pressed, starts the SheetPlan moving through the gait table.

  The application can be terminated with 'q' or [esc]
  """
  import sys
  app = None
  if len(sys.argv)>1:
   # if argument is greater than one
   # if argument is --withRobot then front is going to be x and rear is going to be y according to the values from csv file 
    if sys.argv[1]=='--withRobot' or sys.argv[1]=='-r':
      app = SheetPlanApp(bind=dict(
          x='front/@set_pos', y='rear/@set_pos'), 
        robot=dict(count=2) )
   # if argument is --withScratch, it does cat drawing according to the table
    elif sys.argv[1]=='--withScratch' or sys.argv[1]=='-s':
      app = SheetPlanApp(bind=dict(
          x='>catX', y='>catY'),
        scr={} )
  # in default, output is printed to the terminal from CSV
  #creates an object(interface)
  if app is None:
    app = SheetPlanApp(bind=dict(x="#x ", y="#y "))
  #goes to onStart    
  app.run()

