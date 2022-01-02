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

  def fun( self, phase ):
    """
    This is where we put our periodic behavior. Typically we would send commands like
      self.robot.at.angle.set_angle( (phase-0.5) * 4000 )
    """
    s = sin(phase*2*pi)
    # plots hash with the given spaces
    progress( " "*(int((s+1)*30)) + "#" )

  def onStart( self ):
    # gives default value to start
    self.plan = FunctionCyclePlan(self, self.fun, 24)
    self.period = 1

  def on_K_SPACE(self,evt):
    self.plan.setPeriod(0)
    progress('Paused; "s" to continue')

  def _updatePeriod(self):
    self.plan.setPeriod( self.period )
    progress(f'Period changed to {self.period}')

  def on_K_COMMA(self,evt):
    self.period *= 1.2
    return self._updatePeriod()

  def on_K_PERIOD(self,evt):
    self.period /= 1.2
    return self._updatePeriod()

  def on_K_h(self,evt):
    progress( "HELP: ',' and '.' to change period; ' ' stop; 'q' end program; 'h' this help; 's' to start Plan" )

  def on_K_s(self,evt):
    progress( "Starting cycles...." )
    self._updatePeriod()
    self.plan.start()

if __name__=="__main__":
  print("""
  Demo of FunctionCyclePlan class
  -------------------------------

  When any key is pressed, starts a FunctionCyclePlan that prints
  an ASCII art sine wave under keyboard control.

  h -- help
  q -- quit
  , and . -- change rate
  SPACE -- pause / resume
  s -- start 

  The application can be terminated with 'q' or [esc]
  """)
  app=FunctionCyclePlanApp()
  app.run()

