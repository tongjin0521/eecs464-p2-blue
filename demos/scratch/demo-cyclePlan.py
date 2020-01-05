'''
FILE demo-cyclePlan.py
This file demonstrates the use of cyclePlan which starts an infinite cycle of callbacks
at intervals between 0 and 1 defined by the user.
'''
from joy.decl import *
from joy import JoyApp
from joy.plans import CyclePlan
from joy.misc import curry
#interface
class CyclePlanApp( JoyApp ):
  '''
  This is a concrete class which creates a cycle of callbacks using cyclePlan
  and controls the cycle by receiving user inputs

  It passes a dictionary of callbacks to a cyclePlan in which intervals are the indices
  and callbacks are values.
  The cyclePlan manages the iterations and controls it's period using functions like setPeriod
  The user inputs are manipulated in the onEvent function

  It will be useful for those who want to perform multiple periodic operations
  '''
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,scr={},*arg,**kw)

  def onStart( self ):
    # Define callbacks moving cat to waypoints
    #sensorUpdate is used to move the cat in the scratch window
    def wayPt1(self):
      self.app.scr.sensorUpdate(catX=100,catY=100)
    def wayPt2(self):
      self.app.scr.sensorUpdate(catX=100,catY=-100)
    def wayPt3(self):
      self.app.scr.sensorUpdate(catX=-100,catY=-100)
    def wayPt4(self):
      self.app.scr.sensorUpdate(catX=-100,catY=100)
    #stores the points in a dictionary
    CYCLE = {
      0.0 : wayPt1,
      0.2 : wayPt2,
      0.6 : wayPt3,
      0.8 : wayPt4
    }
    #initializes a plan(thread)
    #The cycle plan does all the backend stuff for the user by using the callbacks defined
    #by the user as arguement
    self.plan = CyclePlan(self, CYCLE)
    self.plan.onStart = curry(progress,">>> START")
    self.plan.onStop = curry(progress,">>> STOP")
    self.freq = 1.0
    self.rate = 0.05
    self.limit = 1/0.45


  def onEvent(self, evt):
    #if any key is pressed
    if evt.type==KEYDOWN:
      #quit
      if evt.key in [K_q,27]: # 'q' and [esc] stop program
        self.stop()
      #changes the period to 0 cycle
      elif evt.key==K_SPACE: # [space] stops cycles
        self.plan.setPeriod(0)
        progress('Period changed to %s' % str(self.plan.period))
      #change the frequency of the plan
      elif evt.key in (K_COMMA, K_PERIOD):
        f = self.freq
        # Change frequency up/down in range -limit..limit hz
        if evt.key==K_COMMA:
          f = (1-self.rate)*f - self.rate*self.limit
        else:
          f = (1-self.rate)*f + self.rate*self.limit
        if abs(f) < 1.0/self.limit:
          self.plan.setPeriod( 0 )
        else:
          self.plan.setPeriod( 1/f )
        self.freq = f
        progress('Period changed to %g, %.2f Hz' % (self.plan.period,f))
       #for help
      elif evt.key==K_h:
        progress( "HELP: ',' to decrease/reverse period; '.' opposite; ' ' stop; 'q' end program; 'h' this help; other keys start Plan" )
      #to start a plan
      else: # any other key
        progress( "Starting cycles...." )
        self.plan.start()
    if evt.type!=TIMEREVENT:
      JoyApp.onEvent(self,evt)

if __name__=="__main__":
  print("""
  Demo of SheetPlan class
  -----------------------

  Use this demo with the demo-plans.sb Scratch project.

  When any key is pressed, starts a SheetPlan making the
  cat move around a square.
  This will basically create a cycle with explicitely declared callbacks
  The application can be terminated with 'q' or [esc]
  """)
  #strats an interface
  app=CyclePlanApp()
  #directs to the onStart function and starts a cycle plan(like an infinite loop)
  app.run()
