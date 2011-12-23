from joy import *

class MultiClickApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
  
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
