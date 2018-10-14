'''
FILE demo-multiclick

This file demonstrates the use of a MultiClick object to detect combination of 
events. It tracks the previous clicks to yield appropriate combination of multiple clicks
'''
from joy.decl import *
from joy.events import *
from joy import JoyApp, progress
from joy.plans import MultiClick

class MultiClickApp( JoyApp ):
  '''
  This is a concrete class which detects multiple clicks such as keyboards presses, joy
  stick input etc using a MultiClick object

  This class uses MultiClick function which handles the operation and sequence of all the buttonUP
  and buttonDOWN events and makes it easier to detect multiple clicks. It takes a set
  of keyboard, mouse and joysttick inputs as arguements and detects whenever multiple 
  click occurs

  It is useful if you want to integrate precise multiclick events to your code  
  '''
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
  #allowEmpty indicates no keys are currently pressed and passes an empty set of events
  def onStart( self ):
    self.mc = MultiClick(self,allowEmpty=True)
    self.mc.start()
    
  def onEvent( self, evt ):
    if evt.type in set([
        KEYDOWN,MOUSEBUTTONDOWN,JOYBUTTONDOWN,
        KEYUP,MOUSEBUTTONUP,JOYBUTTONUP]):
      self.mc.push(evt)
    else:
      JoyApp.onEvent(self,evt)

  def onClick( self, plan, evt ):    
    progress( "Clicked: %s %s" 
      % (MultiClick.nameFor(evt),describeEvt(evt)) )
  
  def onMultiClick( self, plan, evts ):        
    progress( "MultiClick: [%s]" % (", ".join(evts.keys())) )

if __name__=="__main__":
  print """
    Demonstration of the MultiClick plan
    ------------------------------------
    
    Shows the default behavior of the MultiClick plan, which can
    be used to collect <something>UP and <something>DOWN events
    and synthesize them into Click events -- a single, short click
    and MultiClick events -- multiple buttons down at once.
    
    The demonstration runs with allowEmpty=True, which also creates
    MultiClick events when the clicked keys change to an empty set
  """
  app = MultiClickApp()
  app.run()
