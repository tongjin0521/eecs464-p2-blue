'''
This file demonstrates the use of a GaitCyclePlan which performs periodic operations at intervals
defined in a spreadsheet. 
'''
from joy.decl import *
from joy import JoyApp
from joy.misc import curry
from joy.plans import GaitCyclePlan
from joy.misc import loadCSV

class GaitCyclePlanApp( JoyApp ):
  '''
  This is a concrete class which starts and stops a gaitCyclePlan on user input
  
  This class creates a GaitCyclePlan which passes an output object given by the user along with the various cycle
  parameters like frequency limits. It receives user input in onEvent function 
  to start and control the cycle.

  This is useful if you want to execute a cycle defined in a spreadsheet
  '''
  #takes values frm a spreadsheet
  SHEET = loadCSV("M4ModLab.csv")
  
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)

  #initializes a plan based on the arguemnets passed on __init__function  
  def onStart( self ):
    if self.robot is not None:
      self.plan = GaitCyclePlan( self, self.SHEET, maxFreq = 0.3,
        x='front/@set_pos', y='rear/@set_pos')
    else:
      self.plan = GaitCyclePlan( self, self.SHEET, maxFreq = 0.3,
        x='#catX', y='#catY')
    #Curry function creates a callback using it's arguements
    self.plan.onStart = curry(progress,">>> START")
    self.plan.onStop = curry(progress,">>> STOP")
    self.plan.setFrequency(0.2)

  #acts on our inputs
  def onEvent(self, evt):
    if evt.type==KEYDOWN:
      if evt.key in [K_q,27]: # 'q' and [esc] stop program
        self.stop()
        #
      elif evt.key==K_SPACE: # [space] stops cycles
        self.plan.setPeriod(0)
        progress('Stopped motion')
        #
      elif evt.key in (K_COMMA, K_PERIOD):
        f = self.plan.getFrequency()
        # Change frequency up/down in range -limit..limit hz
        if evt.key==K_COMMA:
          f -= 0.03
        else:
          f += 0.03
        self.plan.setFrequency(f)
        progress('Frequency changed to %.2f Hz' % self.plan.getFrequency())
        #
      elif evt.key==K_h:
        progress( "HELP: ',' to decrease/reverse period; '.' opposite; ' ' stop; 'q' end program; 'h' this help; other keys start Plan" )
      #sets a frequency
      elif evt.key in range(K_0,K_9+1):
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
  print("""
  Demo of SheetPlan class
  -----------------------
  When any key is pressed, starts a SheetPlan
  The application can be terminated with 'q' or [esc]
  """)
  import joy
  from sys import argv
  if len(argv)>1 and "robot".startswith(argv[1].lower()):
    app = GaitCyclePlanApp(robot={'count':2})
  else:
    app=GaitCyclePlanApp()
  #directs  to onStart function
  app.run()

