'''
FILE demo-FunctionCyclePlan.py

This file creates an interface to start a functioncycle using our defined function
and controls the cycle on user inputs
'''
from joy import JoyApp
from joy.misc import curry
from joy.decl import *
from math import sin,pi
from joy.plans import FunctionCyclePlan

class FunctionCyclePlanApp( JoyApp ):
  '''
  This is a concrete class

  This class is used to start  a function cycle plan using the function defined in onStart
  and controls the function cycle parameters using user inputs defined in onEvent

  It controls the parameters like frequency using .setPeriod(PeriodValue) method of the FunctionCyclePlan
  which sets the period within some limits defined in function cycle plan and sets a knot in the cycle
  using knot arguement in FunctionCyclePlan

  This is useful for those who want to control a motors speed to have a smooth motion
  '''
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)

  def onStart( self ):
    # gives default value to start
    def fun( phase ):
      s = sin(phase*2*pi)
      # plots hash with the given spaces
      progress( " "*(int((s+1)*30)) + "#" )
    self.plan = FunctionCyclePlan(self, fun, 24)
    #curry function returning a callable with initial parameters already set
    #It basically creates a new function
    self.plan.onStart = curry(progress,">>> START")
    self.plan.onStop = curry(progress,">>> STOP")
    self.freq = 1.0
    self.rate = 0.05
    self.limit = 1/0.45

  def onEvent(self, evt):
    if evt.type==KEYDOWN:
      if evt.key in {K_q,K_ESCAPE}: # 'q' and [esc] stop program
        self.stop()
      elif evt.key==K_SPACE: # [space] stops cycles
        self.plan.setPeriod(0)
        progress('Period changed to %s' % str(self.plan.period))
        #
      elif evt.key in {K_COMMA,K_PERIOD}:
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
        #help message
      elif evt.key==K_h:
        progress( "HELP: ',' to decrease/reverse period; '.' opposite; ' ' stop; 'q' end program; 'h' this help; other keys start Plan" )
        #starts the cycle given any key
      else:
        progress( "Starting cycles...." )
        self.plan.start()
    if evt.type!=TIMEREVENT:
      JoyApp.onEvent(self,evt)

if __name__=="__main__":
  print """
  Demo of FunctionCyclePlan class
  -------------------------------

  When any key is pressed, starts a FunctionCyclePlan that prints
  an ASCII art sine wave under keyboard control.

  h -- help
  q -- quit
  , and . -- change rate
  SPACE -- pause / resume
  any other key -- start

  The application can be terminated with 'q' or [esc]
  """
  app=FunctionCyclePlanApp()
  app.run()
