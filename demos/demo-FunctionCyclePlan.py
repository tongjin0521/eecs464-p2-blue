from joy import *
from math import sin,pi

class FunctionCyclePlanApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
    
  def onStart( self ):
    def fun( phase ):
      s = sin(phase*2*pi)
      progress( " "*(int((s+1)*30)) + "#" )   
    self.plan = FunctionCyclePlan(self, fun, 24)
    self.plan.onStart = curry(progress,">>> START")
    self.plan.onStop = curry(progress,">>> STOP")
    self.freq = 1.0
    self.rate = 0.05
    self.limit = 1/0.45
      
  def onEvent(self, evt):
    if evt.type==KEYDOWN:
      if evt.key in [ord('q'),27]: # 'q' and [esc] stop program
        self.stop()
        #
      elif evt.key==ord(' '): # [space] stops cycles
        self.plan.setPeriod(0)
        progress('Period changed to %s' % str(self.plan.period))
        #
      elif evt.key in (ord(','),ord('.')):
        f = self.freq
        # Change frequency up/down in range -limit..limit hz
        if evt.key==ord(','):
          f = (1-self.rate)*f - self.rate*self.limit
        else:
          f = (1-self.rate)*f + self.rate*self.limit
        if abs(f) < 1.0/self.limit:
          self.plan.setPeriod( 0 )
        else:
          self.plan.setPeriod( 1/f )
        self.freq = f
        progress('Period changed to %g, %.2f Hz' % (self.plan.period,f))
        #
      elif evt.key==ord('h'):
        progress( "HELP: ',' to decrease/reverse period; '.' opposite; ' ' stop; 'q' end program; 'h' this help; other keys start Plan" )
        #
      else: # any other key
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
  import joy
  joy.DEBUG[:]=[]
  app=FunctionCyclePlanApp()
  app.run()

